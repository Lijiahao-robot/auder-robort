交付检查结果

verification_report.json：10 项无窗口物理检查全部通过。
demo_report.json / demo_trajectory.csv：49 秒自动动作演示，未跌倒。
direct_walk_report.json：从启动开始直接行走 30 秒，完成 9 步，未跌倒。
atlas_preview.png：真实 MuJoCo 3.12.0 对模型挥手状态的渲染。
preview_report.json：渲染时对应的物理状态。
runtime_environment.json：实际测试环境与源码哈希。

run_report.json / trajectory.csv 会在用户运行程序后生成或更新。
Windows 原生图形窗口和批处理未在 Windows 实机测试。

model_loading_verification.json：内存模型加载检查，中文/空格路径中启动并行走 15 秒，缺失 XML/STL 的提示检查。测试平台为 Linux。
