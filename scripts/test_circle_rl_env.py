#!/usr/bin/env python3
"""B2-S0 Isaac environment tests only: no PPO, optimizer, ROS, CAN or SDK."""

import argparse
import faulthandler
import json
import math
from pathlib import Path
import platform
import subprocess
import sys
import traceback

import numpy as np

REPO=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(REPO/'src'))
sys.path.insert(0,str(REPO/'scripts'))
from validate_sim_robot import digest, write_json


def clean(value):
    if isinstance(value,dict): return {k:clean(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)): return [clean(v) for v in value]
    if isinstance(value,float) and not math.isfinite(value): return None
    return value


def main():
    from isaaclab.app import AppLauncher
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf',type=Path,required=True)
    parser.add_argument('--usd',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--config',type=Path,default=REPO/'configs/b2_s0_circle_sim_only.json')
    parser.add_argument('--trajectory',type=Path,default=REPO/'runs/b1_s1/plan_005/trajectory.jsonl')
    parser.add_argument('--controller',choices=('pd','pd_gravity'),default='pd')
    parser.add_argument('--num_envs',type=int,default=2)
    parser.add_argument('--test-seed',type=int,default=20261113)
    parser.add_argument('--quick',action='store_true')
    AppLauncher.add_app_launcher_args(parser)
    args=parser.parse_args()
    if args.device!='cpu': raise ValueError('B2-S0 tests require --device cpu')
    args.output.mkdir(parents=True,exist_ok=False)
    task=json.loads(args.config.read_text()); task['controller_mode']=args.controller
    write_json(args.output/'input_config.json',task)
    write_json(args.output/'command.json',{'argv':sys.argv,'parameter_class':'SIM_ONLY'})
    app=env=None; failures=[]; checks={}; records=[]; metadata={}
    last_observation={}; continuity={'compared_transitions':0,'maximum_abs_step_change':0.0}
    try:
        app=AppLauncher(args).app
        import torch
        import isaaclab
        from isaacsim.core.version import get_version
        from robot_rl_sim2real.isaac_circle_env import CircleEnvCfg,CircleTrackingEnv
        from robot_rl_sim2real.circle_rl import reward_components
        cfg=CircleEnvCfg(seed=args.test_seed)
        cfg.scene.num_envs=args.num_envs
        cfg.scene.env_spacing=task['environment_spacing_m']
        cfg.decimation=task['decimation']; cfg.sim.dt=task['physics_dt_s']
        cfg.episode_length_s=task['episode_duration_s']
        lab_root=Path(isaaclab.__file__).resolve().parents[3]
        version=get_version()[0]
        commit=subprocess.check_output(['git','-C',str(lab_root),'rev-parse','HEAD'],text=True).strip()
        if version!='4.5.0' or commit!='90b79bb2d44feb8d833f260f2bf37da3487180ba':
            raise RuntimeError('Isaac environment differs from B1-S1')
        env=CircleTrackingEnv(cfg,task,args.urdf,args.usd,args.output)
        metadata={'python':platform.python_version(),'torch':torch.__version__,'numpy':np.__version__,
                  'isaac_sim':version,'isaac_lab_commit':commit,'isaac_lab_extension':isaaclab.__version__,
                  'controller_mode':args.controller,'parameter_class':'SIM_ONLY','seed':args.test_seed,
                  'num_envs':env.num_envs,'physics_dt_s':env.physics_dt,'control_dt_s':env.step_dt,
                  'maximum_episode_steps':env.max_episode_length,'finite_horizon':env.cfg.is_finite_horizon,
                  'physx_joint_order':env.robot.joint_names,
                  'canonical_indices':env.indices,'urdf_sha256':digest(args.urdf),'usd_layer_sha256':env.usd_hashes,
                  'source_sha256':{p:digest(REPO/p) for p in ('scripts/test_circle_rl_env.py',
                    'src/robot_rl_sim2real/isaac_circle_env.py','src/robot_rl_sim2real/circle_rl.py',
                    'configs/b2_s0_circle_sim_only.json','scripts/run_circle_trajectory.py','scripts/check_model_collisions.py')},
                  'collision_filter_scope':'cross-environment only; intra-robot self collision enabled',
                  'training_started':False,'hardware_connected':False}

        def step(action,label):
            obs,r,terminated,truncated,_=env.step(torch.as_tensor(action,dtype=torch.float32,device=env.device))
            assert obs['policy'].shape==(env.num_envs,31)
            assert torch.isfinite(obs['policy']).all() and torch.isfinite(r).all()
            for row in env.last_transition:
                assert abs(row['reward']['total']-sum(row['reward'][k] for k in ('tracking','action','action_change')))<1e-10
                assert 0<row['reward']['total']<=1.031/env.max_episode_length
                assert np.max(np.abs(row['applied_action']))<=1+1e-6
                assert np.all(np.array(row['q_target_rad'])>=env.lower-1e-6)
                assert np.all(np.array(row['q_target_rad'])<=env.upper+1e-6)
                i=row['env_id']
                if not row['terminated'] and not row['truncated']:
                    measured=np.asarray(row['q_rad']); velocity=np.asarray(row['dq_rad_s'])
                    expected=np.concatenate(((measured-env.home)/task['local_joint_radius_rad'],
                        velocity/task['velocity_observation_scale_rad_s'],
                        (np.asarray(row['reference_position_m'])-row['position_m'])/task['position_observation_scale_m'],
                        np.asarray(row['reference_velocity_m_s'])/(env.circle['radius_m']*2*math.pi/env.circle['period_s']),
                        row['applied_action'],(np.asarray(row['q_target_rad'])-measured)/task['target_observation_scale_rad'],
                        [row['episode_time_s']/task['episode_duration_s']]))
                    observed=obs['policy'][i].cpu().numpy()
                    np.testing.assert_allclose(observed,expected,atol=1e-6,rtol=1e-6)
                    old=last_observation.get(i)
                    if old is not None and old[0]==label and old[1]+1==row['step']:
                        delta=float(np.max(np.abs(observed-old[2])))
                        assert delta < 3, ('observation discontinuity',label,i,row['step'],delta)
                        continuity['compared_transitions']+=1
                        continuity['maximum_abs_step_change']=max(continuity['maximum_abs_step_change'],delta)
                    last_observation[i]=(label,row['step'],observed.copy())
                else:
                    last_observation.pop(i,None)
                records.append({'test':label,**clean(row)})
            return obs,r,terminated,truncated

        zeros=np.zeros((env.num_envs,6))
        obs,_=env.reset(seed=args.test_seed)
        assert obs['policy'].shape==(env.num_envs,31) and obs['policy'].dtype==torch.float32 and env.cfg.is_finite_horizon
        before=obs['policy'].clone(); old_target=env.target.copy(); env.target+=.01
        changed=env._get_observations()['policy']
        assert torch.equal(before[:,:24],changed[:,:24]) and not torch.equal(before[:,24:30],changed[:,24:30])
        assert torch.equal(before[:,30],changed[:,30])
        env.target=old_target
        checks['observation']={'status':'PASS','shape':list(before.shape),'target_error_observed':True,'privileged_critic':False}

        pre_q,_=env._state()
        _,_,term,_=step(np.ones_like(zeros),'action_boundary')
        assert not term.any()
        for i,row in enumerate(env.last_transition):
            np.testing.assert_allclose(np.asarray(row['q_target_rad'])-pre_q[i],.00075,atol=2e-8)
        checks['action_boundary']={'status':'PASS','first_increment_rad':.00075,'maximum_normalized_action':1.0}

        for _ in range(4 if args.quick else 24):
            env.reset()
            assert np.all(np.abs(env._state()[0]-env.home)<=task['reset_joint_noise_rad']+1e-7)
            assert np.all(env.episode_length_buf.cpu().numpy()==0) and np.all(env.current_action==0)
            _,_,term,trunc=step(zeros,'reset_physics')
            assert not term.any() and not trunc.any()
        checks['reset']={'status':'PASS','sample_count':len(env.reset_records),
                         'minimum_jacobian_sigma':min(r['jacobian_sigma'] for r in env.reset_records),
                         'maximum_initial_position_error_m':max(r['position_error_m'] for r in env.reset_records)}

        def oracle():
            plan=[json.loads(line) for line in args.trajectory.read_text().splitlines()]
            expected=json.loads((REPO/task['baseline_summary']).read_text())['trajectory_sha256']
            if digest(args.trajectory)!=expected: raise ValueError('unreviewed diagnostic IK reference')
            metadata['diagnostic_trajectory_sha256']=expected
            times=np.array([r['time_s'] for r in plan]); qs=np.array([r['q_target_rad'] for r in plan])
            env.reset(seed=args.test_seed)
            for k in range(32 if args.quick else env.max_episode_length):
                desired=np.array([np.interp(min(14.0,float(t)*env.step_dt+4.0+env.step_dt),times,qs[:,j])
                                  for t in env.episode_length_buf for j in range(6)]).reshape(env.num_envs,6)
                action=np.clip((desired-env.target)/task['action_scale_rad'],-1,1)
                _,_,term,trunc=step(action,'diagnostic_ik')
                assert not term.any(),env.last_transition
                if not args.quick:
                    assert bool(trunc.all())==(k==env.max_episode_length-1)
            subset=[r for r in records if r['test']=='diagnostic_ik']
            circle=[r for r in subset if r['reference_segment']=='circle']
            if not args.quick:
                assert {r['reference_segment'] for r in subset}=={'startup','circle','stop','hold'}
                one_env=[r for r in circle if r['env_id']==0]
                assert abs(one_env[-1]['reference_phase_rad']-one_env[0]['reference_phase_rad']-2*math.pi)<1e-6
            checks['episode_time_and_diagnostic_ik']={'status':'PASS','control_steps':k+1,
                'timeout_observed':bool(trunc.all()),'duration_s':env.last_transition[0]['episode_time_s'],
                'actual_duration_s':env.last_transition[0]['actual_elapsed_simulation_s'],
                'maximum_fk_position_error_m':max(r['fk_position_error_m'] for r in subset),
                'maximum_orientation_error_rad':max(r['orientation_error_rad'] for r in subset),
                'circle_rmse_m':float(np.sqrt(np.mean([r['position_error_m']**2 for r in circle]))) if circle else None,
                'return_by_env':[sum(r['reward']['total'] for r in subset if r['env_id']==i) for i in range(env.num_envs)],
                'reference_segments':sorted({r['reference_segment'] for r in subset})}
        oracle()

        if not args.quick:
            def policy_test(label,random):
                env.reset(seed=args.test_seed)
                rng=np.random.default_rng(args.test_seed+99)
                first=[None]*env.num_envs; sums=np.zeros(env.num_envs); errors=[[] for _ in range(env.num_envs)]
                for k in range(env.max_episode_length):
                    _,_,term,trunc=step(rng.uniform(-1,1,zeros.shape) if random else zeros,label)
                    for i,row in enumerate(env.last_transition):
                        if first[i] is None:
                            sums[i]+=row['reward']['total']
                            errors[i].append(row['position_error_m'])
                            if term[i] or trunc[i]: first[i]={'steps':k+1,'return':float(sums[i]),
                                'position_rmse_m':float(np.sqrt(np.mean(np.square(errors[i])))),
                                'reasons':row['termination_reasons'],'truncated':bool(trunc[i])}
                    if not random and all(x is not None for x in first): break
                checks[label]={'status':'PASS','tested_control_steps':k+1,'first_episodes':first,
                               'termination_count':sum(r['terminated'] for r in records if r['test']==label)}
            policy_test('zero_action',False)
            policy_test('random_action',True)
            for i,diagnostic_return in enumerate(checks['episode_time_and_diagnostic_ik']['return_by_env']):
                assert diagnostic_return > checks['zero_action']['first_episodes'][i]['return']
                assert diagnostic_return > checks['random_action']['first_episodes'][i]['return']
            checks['stationary_and_failure_reward_comparison']={'status':'PASS',
                'criterion':'diagnostic IK return exceeds both zero-action and first random episode returns'}

            actions=np.random.default_rng(args.test_seed+7).uniform(-.25,.25,(32,env.num_envs,6))
            def repeat(label):
                env.reset(seed=args.test_seed)
                result=[]
                for a in actions:
                    obs,r,term,trunc=step(a,label)
                    result.append(np.concatenate((obs['policy'].cpu().numpy().ravel(),r.cpu().numpy(),term.cpu().numpy(),trunc.cpu().numpy())))
                return np.array(result)
            first,second=repeat('repeat_a'),repeat('repeat_b')
            difference=float(np.max(np.abs(first-second)))
            assert difference<1e-7,difference
            env.reset(seed=args.test_seed); q_seed=env._state()[0].copy()
            env.reset(seed=args.test_seed+1)
            assert not np.array_equal(q_seed,env._state()[0])
            checks['seed_repeatability']={'status':'PASS','steps':len(actions),'maximum_absolute_difference':difference,'different_seed_changes_reset':True}

            if env.num_envs>1:
                env.reset(seed=args.test_seed)
                step(np.full_like(zeros,.05),'independence_setup')
                q,dq=env._state(); ref=env.reference(float(env.episode_length_buf[1])*env.step_dt)
                saved=[q[1].copy(),dq[1].copy(),env.current_action[1].copy(),env.target[1].copy(),int(env.episode_length_buf[1]),env.rngs[1].bit_generator.state]
                env._reset_idx(torch.tensor([0],device=env.device)); env.scene.write_data_to_sim(); env.sim.forward()
                np.testing.assert_array_equal(env._state()[0][1],saved[0]); np.testing.assert_array_equal(env._state()[1][1],saved[1])
                np.testing.assert_array_equal(env.current_action[1],saved[2]); np.testing.assert_array_equal(env.target[1],saved[3])
                assert int(env.episode_length_buf[1])==saved[4] and env.rngs[1].bit_generator.state==saved[5]
                np.testing.assert_array_equal(env.reference(float(env.episode_length_buf[1])*env.step_dt)['position_m'],ref['position_m'])
                checks['selective_reset_independence']={'status':'PASS','other_env_state_target_action_time_rng_unchanged':True}

            def write_pose(q):
                ids=torch.tensor([0],device=env.device)
                physical=env.robot.data.joint_pos[ids].clone()
                physical[:,env.indices]=torch.tensor(q,dtype=torch.float32,device=env.device)
                env.robot.write_joint_state_to_sim(physical,torch.zeros_like(physical),env_ids=ids)
                env.scene.write_data_to_sim(); env.sim.forward()

            env.reset(seed=args.test_seed); step(zeros,'isolation_control')
            other_control=env.last_transition[1] if env.num_envs>1 else None
            env.reset(seed=args.test_seed)
            fixture=json.loads((REPO/'configs/b1_s1_collision_sim_only.json').read_text())
            folded=fixture['positive_q_rad']
            write_pose(folded)
            _,_,term,_=step(zeros,'collision_fault')
            assert term[0] and 'geometry_collision' in env.last_transition[0]['termination_reasons']
            collision_row=env.last_transition[0]
            if env.num_envs>1:
                assert not term[1]
                np.testing.assert_allclose(env.last_transition[1]['q_rad'],other_control['q_rad'],atol=1e-7,rtol=0)
            checks['collision_termination']={'status':'PASS','reasons':collision_row['termination_reasons'],
                'physx_events':env.contact_events,'other_environment_unaffected':env.num_envs>1}

            env.reset(seed=args.test_seed); outside=env.home.copy(); outside[0]=env.limits[0,1]+.01
            write_pose(outside)
            _,_,term,_=step(zeros,'limit_fault')
            assert term[0] and 'joint_limit' in env.last_transition[0]['termination_reasons']
            checks['joint_limit_termination']={'status':'PASS','reasons':env.last_transition[0]['termination_reasons']}

            env.reset(seed=args.test_seed); invalid=zeros.copy(); invalid[0,0]=float('nan')
            _,_,term,_=step(invalid,'nonfinite_action_fault')
            assert term[0] and 'invalid_action' in env.last_transition[0]['termination_reasons']
            checks['invalid_action_termination']={'status':'PASS','finite_returned_observation':True}

            env.reset(seed=args.test_seed)
            original=env._state; calls=[0]
            def corrupt_feedback():
                q,dq=original(); calls[0]+=1
                if calls[0]<=2: q[0,0]=float('nan')
                return q,dq
            env._state=corrupt_feedback
            try: _,_,term,_=step(zeros,'nonfinite_feedback_fault')
            finally: env._state=original
            assert term[0] and 'nonfinite_state' in env.last_transition[0]['termination_reasons']
            checks['nonfinite_feedback_termination']={'status':'PASS','method':'corrupt state-reader output; no NaN written to PhysX','finite_returned_observation':True}

            best=reward_components([0],np.zeros((1,6)),np.zeros((1,6)),env.max_episode_length,task)
            worst=reward_components([.1],np.ones((1,6)),-np.ones((1,6)),env.max_episode_length,task)
            assert float(worst['total'][0])>0 and float(best['total'][0])<.002
            checks['failure_return_accounting']={'status':'PASS','best_possible_one_step_return':float(best['total'][0]),
                'minimum_possible_step_return':float(worst['total'][0]),'perfect_full_horizon_return':float(best['total'][0]*env.max_episode_length),
                'note':'termination handled separately; continuing yields positive per-step reward'}

        assert continuity['compared_transitions']>0
        checks['observation_continuity']={'status':'PASS',**continuity,'scope':'consecutive non-reset observations'}
        metadata['reset_count']=len(env.reset_records)
        metadata['contact_event_count']=len(env.contact_events)
        metadata['cooked_mesh_comparison']=env.cooked_report
    except Exception as exc:
        failures.append(str(exc)); traceback.print_exc()
    finally:
        if env is not None:
            if not all(digest(args.usd.parent/p)==h for p,h in env.usd_hashes.items()): failures.append('source USD changed')
            write_json(args.output/'resets.json',clean(env.reset_records))
        write_json(args.output/'runtime_metadata.json',clean(metadata))
        with (args.output/'transitions.jsonl').open('w') as stream:
            for row in records: stream.write(json.dumps(clean(row),allow_nan=False)+'\n')
        result={'status':'PASS' if not failures else 'BLOCKED','parameter_class':'SIM_ONLY',
                'suite':'quick' if args.quick else 'full','controller_mode':args.controller,
                'checks':checks,'failures':failures,'logged_transitions':len(records),
                'training_started':False,'hardware_connected':False}
        write_json(args.output/'result.json',clean(result))
        print('B2_S0_RESULT='+json.dumps(clean(result),ensure_ascii=False),flush=True)
        if app is not None:
            faulthandler.dump_traceback_later(8,file=sys.stderr)
            from isaaclab.sim import SimulationContext
            active_sim=env.sim if env is not None else SimulationContext.instance()
            if active_sim is not None:
                active_sim._disable_app_control_on_stop_handle=True
                active_sim.stop()
                print('B2_S0_TIMELINE_STOPPED',flush=True)
            if env is not None:
                env.close()
            app.close(wait_for_replicator=False)
            faulthandler.cancel_dump_traceback_later()
            print('B2_S0_SHUTDOWN_OK',flush=True)
    if failures: raise SystemExit(1)


if __name__=='__main__': main()
