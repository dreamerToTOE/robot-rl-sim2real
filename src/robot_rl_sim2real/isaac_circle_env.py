"""B2-S0 local circle DirectRLEnv. Import only AFTER AppLauncher.

No task registration, training runner, SDK or vendor assets are distributed.
shortcut: CPU FK and exact collision checks favor auditable small batches;
benchmark before increasing beyond the tested small environment count.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import re

import carb
import numpy as np
import omni.physx
import omni.usd
from pxr import PhysxSchema, PhysicsSchemaTools, UsdPhysics
import torch

import isaaclab.sim as sim_utils
from isaaclab.actuators import IdealPDActuatorCfg
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass

from check_model_collisions import CollisionModel
from compare_model_fk import error, parse_urdf
from run_circle_trajectory import inspect_and_screen_cooked
from validate_sim_robot import digest, transform
from .circle_rl import map_action, observation, reference_at, reward_components
from .circle_trajectory import numerical_jacobian

REPO = Path(__file__).resolve().parents[2]


@configclass
class CircleEnvCfg(DirectRLEnvCfg):
    decimation = 4
    episode_length_s = 10.5
    is_finite_horizon = True
    observation_space = 31
    action_space = 6
    state_space = 0
    sim: sim_utils.SimulationCfg = sim_utils.SimulationCfg(
        dt=1/240, device="cpu", render_interval=4, use_fabric=False,
        enable_scene_query_support=True,
        physx=sim_utils.PhysxCfg(enable_enhanced_determinism=True),
    )
    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=2, env_spacing=2.0, replicate_physics=False,
    )


class CircleTrackingEnv(DirectRLEnv):
    """Actual-state 31D actor, previous-target position increments, fixed local reference."""

    def __init__(self, cfg, task, urdf, usd, output):
        self.task, self.urdf, self.usd, self.output = task, Path(urdf), Path(usd), Path(output)
        self.output.mkdir(parents=True, exist_ok=True)
        if task["parameter_class"] != "SIM_ONLY" or task["controller_mode"] not in ("pd", "pd_gravity"):
            raise ValueError("unsupported controller or parameter class")
        if cfg.sim.device != "cpu" or not 1 <= cfg.scene.num_envs <= 4:
            raise ValueError("B2-S0 is limited to 1..4 CPU environments")
        if cfg.seed is None:
            raise ValueError("an explicit seed is required")
        if not cfg.is_finite_horizon:
            raise ValueError("the single-circle task requires a finite horizon")
        if cfg.sim.dt != task["physics_dt_s"] or cfg.decimation != task["decimation"]:
            raise ValueError("control timing differs from the recorded task")
        self.baseline = json.loads((REPO / task["baseline_config"]).read_text())
        source = json.loads((REPO / task["baseline_summary"]).read_text())
        self.circle, self.home_pose = source["circle"], np.asarray(source["home_pose_link6"])
        if source["status"] != "PASS" or digest(self.urdf) != self.baseline["source_urdf_sha256"]:
            raise ValueError("unreviewed model or reference")
        self.usd_hashes = json.loads((REPO / "docs/reports/data/B0_R1_STATIC_IMPORT.json").read_text())["generated_usd_sha256"]
        if not all(digest(self.usd.parent / p) == h for p, h in self.usd_hashes.items()):
            raise ValueError("private USD differs from B0-R1A")
        duration = 2*task["circle_ramp_duration_s"] + self.circle["period_s"] + task["stop_hold_duration_s"]
        if abs(duration - cfg.episode_length_s) > 1e-10 or duration != task["episode_duration_s"]:
            raise ValueError("episode and reference durations disagree")
        self.model, self.geometry = parse_urdf(self.urdf), CollisionModel(self.urdf)
        self.names = [j.name for j in self.model.moving]
        if self.names != self.baseline["joint_names"]:
            raise ValueError("URDF joint order differs from the reviewed contract")
        self.home = np.asarray(self.baseline["q_home_rad"])
        self.limits = np.array([[j.limits['lower'], j.limits['upper']] for j in self.model.moving])
        self.lower = np.maximum(self.limits[:, 0]+task['joint_limit_margin_rad'], self.home-task['local_joint_radius_rad'])
        self.upper = np.minimum(self.limits[:, 1]-task['joint_limit_margin_rad'], self.home+task['local_joint_radius_rad'])
        if not np.all(self.lower < self.home) or not np.all(self.home < self.upper):
            raise ValueError("empty local joint interval")
        n = cfg.scene.num_envs
        self.contact_flags = np.zeros(n, dtype=bool)
        self.contact_events, self.reset_records = [], []
        self.pending = [[] for _ in range(n)]
        self.current_action = np.zeros((n, 6))
        self.previous_for_reward = np.zeros((n, 6))
        self.raw_action = np.zeros((n, 6))
        self.target = np.tile(self.home, (n, 1))
        self.last_valid_q = np.tile(self.home, (n, 1))
        self.previous_dq = np.zeros((n, 6))
        self.reset_clock = np.zeros(n)
        self.position_clipped = np.zeros(n, dtype=bool)
        self.rngs = [np.random.default_rng(cfg.seed+1009*i) for i in range(n)]
        self.last_transition = None
        super().__init__(cfg)
        self.indices = [self.robot.joint_names.index(name) for name in self.names]
        if not self.robot.is_fixed_base or set(self.robot.joint_names) != set(self.names):
            raise RuntimeError("fixed base or six-DoF mapping failed")
        if set(self.robot.body_names) != set(self.model.links):
            raise RuntimeError("rigid-body name mapping failed")
        self.base_id = self.robot.body_names.index('base_link')
        self.tip_id = self.robot.body_names.index('link6')
        actual_limits = self.robot.root_physx_view.get_dof_limits()[0].cpu().numpy()[self.indices]
        np.testing.assert_allclose(actual_limits, self.limits, atol=2e-6)
        np.testing.assert_allclose(self.robot.root_physx_view.get_dof_max_forces().cpu().numpy(),
                                   self.baseline['controller']['effort_limit_nm'])
        np.testing.assert_allclose(self.robot.root_physx_view.get_dof_max_velocities().cpu().numpy(),
                                   self.baseline['controller']['velocity_limit_sim_rad_s'])
        if carb.settings.get_settings().get('/physics/disableContactProcessing') is not False:
            raise RuntimeError("contact processing disabled after reset")
        self.cooked_report = inspect_and_screen_cooked(
            omni.usd.get_context().get_stage(), self.geometry,
            [{"sample_index": 0, "q_target_rad": self.home.tolist()}], self.output,
            self.baseline['controller']['contact_offset_m'], '/World/envs/env_0/Robot',
        )
        # Copies must use the same cooked representation, not merely the same USD token.
        cooking = omni.physx.get_physx_cooking_interface()
        for name in self.model.links:
            first = cooking.get_convex_mesh_data(f'/World/envs/env_0/Robot/{name}/collisions', 0)
            for i in range(1, n):
                path = f'/World/envs/env_{i}/Robot/{name}/collisions'
                if cooking.get_nb_convex_mesh_data(path) != 1:
                    raise RuntimeError('unknown clone cooked hull')
                np.testing.assert_allclose(first['vertices'], cooking.get_convex_mesh_data(path, 0)['vertices'], atol=1e-10, rtol=0)

    def _setup_scene(self):
        # DirectRLEnv has already constructed SimulationContext at this point.
        carb.settings.get_settings().set_bool('/physics/disableContactProcessing', False)
        c = self.baseline['controller']
        self.robot = Articulation(ArticulationCfg(
            prim_path='/World/envs/env_.*/Robot', articulation_root_prim_path='/root_joint',
            spawn=sim_utils.UsdFileCfg(usd_path=str(self.usd.resolve()),
                articulation_props=sim_utils.ArticulationRootPropertiesCfg(enabled_self_collisions=True,
                    solver_position_iteration_count=c['solver_position_iterations'],
                    solver_velocity_iteration_count=c['solver_velocity_iterations'])),
            init_state=ArticulationCfg.InitialStateCfg(joint_pos=dict(zip(self.names, self.home.tolist())), joint_vel={'.*': 0.0}),
            actuators={'sim_only_pd': IdealPDActuatorCfg(joint_names_expr=self.names,
                stiffness=c['stiffness_nm_rad'], damping=c['damping_nm_s_rad'],
                effort_limit=c['effort_limit_nm'], effort_limit_sim=c['effort_limit_nm'],
                velocity_limit_sim=c['velocity_limit_sim_rad_s'],
                armature=c['armature_kg_m2'], friction=c['friction_coefficient'])},
        ))
        self.scene.clone_environments(copy_from_source=True)
        # CPU isolation excludes different envs only; all intra-robot pairs remain active.
        self.scene.filter_collisions(global_prim_paths=[])
        self.scene.articulations['robot'] = self.robot
        stage = omni.usd.get_context().get_stage()
        for i in range(self.cfg.scene.num_envs):
            root = stage.GetPrimAtPath(f'/World/envs/env_{i}/Robot/root_joint')
            if PhysxSchema.PhysxArticulationAPI(root).GetEnabledSelfCollisionsAttr().Get() is not True:
                raise RuntimeError('self collision disabled')
            for name in self.model.links:
                prim = stage.GetPrimAtPath(f'/World/envs/env_{i}/Robot/{name}')
                PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0.0)
                collider = stage.GetPrimAtPath(str(prim.GetPath())+'/collisions')
                if not UsdPhysics.CollisionAPI(collider).GetCollisionEnabledAttr().Get():
                    raise RuntimeError('collider disabled')
                if UsdPhysics.MeshCollisionAPI(collider).GetApproximationAttr().Get() != 'convexHull':
                    raise RuntimeError('unexpected collision approximation')
                api = PhysxSchema.PhysxCollisionAPI.Apply(collider)
                api.CreateContactOffsetAttr().Set(c['contact_offset_m'])
                api.CreateRestOffsetAttr().Set(c['rest_offset_m'])
        self.subscription = omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(self._on_contact)

    def _on_contact(self, headers, data):
        for h in headers:
            if not h.num_contact_data:
                continue
            actors = [str(PhysicsSchemaTools.intToSdfPath(x)) for x in (h.actor0, h.actor1)]
            ids = sorted({int(m.group(1)) for actor in actors if (m := re.search(r'/envs/env_(\d+)/', actor))})
            for i in ids:
                self.contact_flags[i] = True
            self.contact_events.append({'actors': actors, 'env_ids': ids, 'simulation_time_s': self.sim.current_time,
                                        'contact_count': h.num_contact_data})

    def reference(self, time_s):
        return reference_at(time_s, self.circle, self.task['circle_ramp_duration_s'], self.task['stop_hold_duration_s'])

    def assess(self, q, dq, reference, contact=False, acceleration=None):
        """Independent safety predicate, also used before accepting reset samples."""
        reasons, metrics = [], {}
        if not np.isfinite(q).all() or not np.isfinite(dq).all():
            return ['nonfinite_state'], metrics
        if np.any(q < self.limits[:, 0]-1e-6) or np.any(q > self.limits[:, 1]+1e-6):
            reasons.append('joint_limit')
        if np.any(q < self.lower-1e-6) or np.any(q > self.upper+1e-6):
            reasons.append('local_joint_bound')
        if np.max(np.abs(dq)) > self.task['observed_velocity_stop_rad_s']:
            reasons.append('velocity')
        if acceleration is not None and (not np.isfinite(acceleration).all() or np.max(np.abs(acceleration)) > self.task['observed_acceleration_stop_rad_s2']):
            reasons.append('acceleration')
        pose = np.asarray(self.model.fk(dict(zip(self.names, q)))['link6'])
        metrics['position_error_m'] = float(np.linalg.norm(pose[:3,3]-reference['position_m']))
        metrics['orientation_error_rad'] = error(pose.tolist(), self.home_pose.tolist())['orientation_rad']
        if metrics['position_error_m'] > self.task['position_error_stop_m']:
            reasons.append('tracking_workspace')
        if metrics['orientation_error_rad'] > self.task['orientation_error_stop_rad']:
            reasons.append('orientation')
        metrics['jacobian_sigma'] = float(np.linalg.svd(numerical_jacobian(self.model, self.names, q), compute_uv=False)[-1])
        if metrics['jacobian_sigma'] < self.task['minimum_jacobian_sigma']:
            reasons.append('singularity')
        if contact:
            reasons.append('physx_contact')
        if 'joint_limit' not in reasons:
            try:
                pairs = self.geometry.check(dict(zip(self.names, q)), aabb_fast=True,
                                            padding_m=self.baseline['controller']['contact_offset_m'])
                if not all(p['separated'] for p in pairs):
                    reasons.append('geometry_collision')
            except RuntimeError:
                reasons.append('collision_unknown')
        metrics['position_m'] = pose[:3,3].tolist()
        return reasons, metrics

    def _state(self):
        return (self.robot.data.joint_pos[:,self.indices].cpu().numpy().astype(float),
                self.robot.data.joint_vel[:,self.indices].cpu().numpy().astype(float))

    def _pre_physics_step(self, actions):
        raw = actions.detach().cpu().numpy()
        if raw.shape != (self.num_envs, 6):
            raise ValueError('expected (num_envs,6) actions')
        q,dq = self._state()
        self.raw_action = raw.copy()
        self.previous_for_reward = self.current_action.copy()
        for i in range(self.num_envs):
            reasons,_ = self.assess(q[i],dq[i],self.reference(float(self.episode_length_buf[i])*self.step_dt),self.contact_flags[i])
            self.pending[i].extend(reasons)
        safe_q = np.clip(np.where(np.isfinite(q), q, self.last_valid_q), self.lower, self.upper)
        mapped = map_action(raw, self.current_action, self.target, self.lower, self.upper, self.task)
        self.current_action, self.target = mapped['effective'], mapped['target']
        self.position_clipped = mapped['position_clipped']
        for i in range(self.num_envs):
            if mapped['invalid'][i]:
                self.pending[i].append('invalid_action')
            if self.pending[i]:
                self.target[i] = safe_q[i]
                self.current_action[i] = 0

    def _apply_action(self):
        target = torch.as_tensor(self.target,device=self.device,dtype=torch.float32)
        self.robot.set_joint_position_target(target,joint_ids=self.indices)
        self.robot.set_joint_velocity_target(torch.zeros_like(target),joint_ids=self.indices)
        gravity = self.robot.root_physx_view.get_gravity_compensation_forces().clone() if self.task['controller_mode']=='pd_gravity' else torch.zeros_like(target)
        if not torch.isfinite(gravity).all():
            raise RuntimeError('nonfinite gravity feedforward')
        self.robot.set_joint_effort_target(gravity)

    def _get_dones(self):
        q,dq=self._state()
        self.checked_q,self.checked_dq=q,dq
        self.checked_ddq=(dq-self.previous_dq)/self.step_dt
        self.previous_dq=dq.copy()
        self.checked_references=[]; self.checked_metrics=[]; self.checked_reasons=[]
        for i in range(self.num_envs):
            t=float(self.episode_length_buf[i])*self.step_dt
            ref=self.reference(t)
            reasons,metrics=self.assess(q[i],dq[i],ref,self.contact_flags[i],self.checked_ddq[i])
            reasons=list(dict.fromkeys(self.pending[i]+reasons))
            if abs(self.sim.current_time-self.reset_clock[i]-t)>1e-5:
                reasons.append('episode_clock')
            if 'nonfinite_state' not in reasons:
                poses=self.robot.data.body_link_pose_w[i].cpu().numpy()
                actual=np.linalg.inv(transform(poses[self.base_id])) @ transform(poses[self.tip_id])
                cpu=self.model.fk(dict(zip(self.names,q[i])))['link6']
                fk=error(actual.tolist(),cpu)
                metrics['fk_position_error_m'],metrics['fk_orientation_error_rad']=fk['position_m'],fk['orientation_rad']
                if fk['position_m']>1e-4 or fk['orientation_rad']>1e-3:
                    reasons.append('fk_mapping')
            if not reasons:
                self.last_valid_q[i] = q[i]
            self.checked_references.append(ref); self.checked_metrics.append(metrics); self.checked_reasons.append(reasons)
        failed=torch.tensor([bool(r) for r in self.checked_reasons],device=self.device)
        timeout=(self.episode_length_buf>=self.max_episode_length) & ~failed
        return failed,timeout

    def _get_rewards(self):
        errors=[m.get('position_error_m',self.task['tracking_reward_scale_m']) for m in self.checked_metrics]
        components=reward_components(errors,self.current_action,self.previous_for_reward,self.max_episode_length,self.task)
        records=[]
        for i in range(self.num_envs):
            ref=self.checked_references[i]
            records.append({'env_id':i,'step':int(self.episode_length_buf[i]),
                'episode_time_s':float(self.episode_length_buf[i])*self.step_dt,
                'actual_elapsed_simulation_s':self.sim.current_time-self.reset_clock[i],
                'absolute_simulation_s':self.sim.current_time,'reference_segment':ref['segment'],
                'reference_phase_rad':ref['phase_rad'],'reference_position_m':ref['position_m'].tolist(),
                'reference_velocity_m_s':ref['velocity_m_s'].tolist(),
                'q_rad':self.checked_q[i].tolist(),'dq_rad_s':self.checked_dq[i].tolist(),
                'ddq_rad_s2':self.checked_ddq[i].tolist(),'q_target_rad':self.target[i].tolist(),
                'raw_action':np.where(np.isfinite(self.raw_action[i]),self.raw_action[i],None).tolist(),
                'previous_action':self.previous_for_reward[i].tolist(),'applied_action':self.current_action[i].tolist(),
                'position_clipped':bool(self.position_clipped[i]),
                'command_effort_nm':self.robot.data.applied_torque[i,self.indices].cpu().tolist(),
                'reward':{key:float(value[i]) for key,value in components.items()},
                'terminated':bool(self.reset_terminated[i]),'truncated':bool(self.reset_time_outs[i]),
                'termination_reasons':self.checked_reasons[i],**self.checked_metrics[i]})
        self.last_transition=records
        self.extras['transition']=records
        self.extras['controller_mode']=self.task['controller_mode']
        return torch.tensor(components['total'],device=self.device,dtype=torch.float32)

    def _get_observations(self):
        q,dq=self._state(); positions=[]; refs=[]
        for i in range(self.num_envs):
            positions.append(np.asarray(self.model.fk(dict(zip(self.names,q[i])))['link6'])[:3,3])
            refs.append(self.reference(float(self.episode_length_buf[i])*self.step_dt))
        obs=observation(q,dq,np.asarray(positions),np.array([r['position_m'] for r in refs]),
                        np.array([r['velocity_m_s'] for r in refs]),self.current_action,self.target,
                        self.episode_length_buf.cpu().numpy()*self.step_dt,self.home,self.task,self.circle)
        return {'policy':torch.as_tensor(obs,device=self.device)}

    def _reset_idx(self, env_ids):
        super()._reset_idx(env_ids)
        for i in env_ids.tolist():
            for attempt in range(self.task['reset_attempts']):
                q=self.home+self.rngs[i].uniform(-self.task['reset_joint_noise_rad'],self.task['reset_joint_noise_rad'],6)
                reasons,metrics=self.assess(q,np.zeros(6),self.reference(0))
                if not reasons:
                    break
            else:
                raise RuntimeError('no valid reset sample; refusing unchecked fallback')
            tensor=torch.as_tensor(q[None,:],device=self.device,dtype=torch.float32)
            ids=torch.tensor([i],device=self.device,dtype=torch.long)
            physical=self.robot.data.default_joint_pos[ids].clone(); physical[:,self.indices]=tensor
            self.robot.write_joint_state_to_sim(physical,torch.zeros_like(physical),env_ids=ids)
            self.robot.set_joint_position_target(tensor,joint_ids=self.indices,env_ids=ids)
            self.robot.set_joint_velocity_target(torch.zeros_like(tensor),joint_ids=self.indices,env_ids=ids)
            self.robot.set_joint_effort_target(torch.zeros_like(physical),env_ids=ids)
            self.current_action[i]=0; self.previous_for_reward[i]=0; self.previous_dq[i]=0
            self.raw_action[i]=0; self.target[i]=q; self.last_valid_q[i]=q; self.pending[i]=[]; self.contact_flags[i]=False
            self.position_clipped[i]=False; self.reset_clock[i]=self.sim.current_time
            self.reset_records.append({'env_id':i,'attempts':attempt+1,'q_rad':q.tolist(),
                                       'initial_phase_rad':self.reference(0)['phase_rad'],**metrics})

    def reset(self, *, seed=None, options=None):
        if seed is not None:
            self.rngs=[np.random.default_rng(seed+1009*i) for i in range(self.num_envs)]
        return super().reset(seed=seed,options=options)

    def close(self):
        self.subscription=None
        super().close()
