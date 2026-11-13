# 将工程骨架发布到 GitHub

当前 GitHub 连接可以访问已有仓库并提交文件，但**不提供创建新仓库的操作**。因此需要仓库所有者先在 GitHub 网页端创建一次空仓库。

## 只需要一次的操作

1. 登录 GitHub，在 `dreamerToTOE` 账号中创建 `robot-rl-sim2real`。
2. 推荐 `Private`。**不要**选择初始化 README、`.gitignore` 或 LICENSE，因为本压缩包已经包含基础文件。
3. 本地解压 `robot-rl-sim2real.zip`，进入 `robot-rl-sim2real/` 文件夹。
4. 执行：

```bash
git init -b main
git add .
git commit -m "chore: scaffold minimal PPO Sim2Real trajectory-tracking project"
git remote add origin https://github.com/dreamerToTOE/robot-rl-sim2real.git
git push -u origin main
```

Git 会要求使用你已配置好的凭证（SSH 或 HTTPS）。这里不会获取或保存你的 token。

也可以仅在 GitHub 建立空仓库后，在 ChatGPT 中告诉我仓库已创建；届时我可以通过当前 GitHub 连接将文件写入该仓库并验证提交。两种方式选其一，不必重复提交。

**注意**：厂商 `student_ros.zip` 不在该压缩包中，也不应该在许可证确认前上传公开 GitHub。
