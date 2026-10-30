# ELE Scout 源码导出说明（2026-10-30）

这是 ELE Scout 当前工作区的完整源码快照，包含最新高程多帧融合、射线清除和 HT 相关修改及模型。
这些修改已包含在源码中，无需再应用配套 Autoware 包内的 ELE 补丁。

本包不含 .git 历史、编译产物、日志、录包和运行验证数据，需要安装依赖并重新构建。
先解压，再将源码提交到 ELE Scout 对应的 GitHub 仓库；更新已有仓库时保留其 .git 历史。
Autoware 和 ELE Scout 应分别提交到各自仓库。

SOURCE_SNAPSHOT.json 记录基线提交和快照信息，MANIFEST_SHA256.txt 记录文件校验和。
配套 Autoware 源码包中的 repositories/scout-external.repos 仍指向旧基线；
上传 ELE 后，将该文件的 version 更新为实际上传的新提交 SHA。
构建后，可用 SCOUT_HT_ELE_WORKSPACE 指定此 ELE 工作区路径。

最新相关验证为 66 项 ELE/CUDA 测试和 93 项 Scout 回归，共 159 项。
现场走人复测主增高区域 69 格恢复，5 个零散格仍有当前邻近回波，需继续核对输入。
这些记录不代表自动驾驶整车验收完成。文档中的 validation/ 运行数据未随包导出。
