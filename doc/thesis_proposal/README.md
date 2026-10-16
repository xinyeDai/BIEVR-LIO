# 硕士开题报告

题目：**面向铁路场景的列车轻量语义地图构建与重定位方法研究**

[proposal.tex](proposal.tex) 为 UTF-8 编码的中文 LaTeX 单文件，正文超过 4000 字，含封面、摘要、目录、技术路线图、公式、实验方案、论文章节、12 个月进度安排及 21 条参考文献。参考文献与 TikZ 图均已内嵌，不需要外部图片或 BibTeX。

## 编译与修改

使用支持中文的 TeX Live / MiKTeX，或上传到 Overleaf 并选择 **XeLaTeX**。

在当前目录执行两次编译，以更新目录及交叉引用：

~~~bash
xelatex -interaction=nonstopmode -halt-on-error proposal.tex
xelatex -interaction=nonstopmode -halt-on-error proposal.tex
~~~

源文件使用 ctexart 与 Fandol 字体。封面姓名、学号、学院、专业、导师通过文件开头的命令修改；日期默认使用编译日期。学校若另有格式要求，可迁移正文及 thebibliography 到学校模板。

编写环境未安装 XeLaTeX，因此本次进行源文件结构、引用完整性与字数检查，未生成或声称已编译 PDF。

## 与仓库的对应关系

报告依据当前工作区内容撰写，未修改算法或用户已有实验结果。前期基础与拟研究内容作了区分：

| 内容 | 主要代码依据 | 报告中的定位 |
|---|---|---|
| 定向高度图与起伏配准 | BIEVR/src/bievr_map.cpp、ls_optimizer.cpp、preprocess.cpp | 原始 BIEVR-LIO 方法基础，引用 RSS 2026 原论文；铁路适配和方向性质量分析作为拟研究内容 |
| 激光惯性流程 | BIEVR/src/pipeline.cpp | 惯性预测、去畸变、激光配准、惯性窗口更新；不将现有实现称为全状态联合紧耦合 |
| GNSS 优化和地图重建 | modules/pgo/src/gnss_optimizer.cpp、interfaces/ros1/src/gnss_optimize.cpp | 已有原型；高程趋势复核和系统化质量赋权作为拟研究内容 |
| 支柱、轨道提取 | interfaces/ros1/src/pole_extractor.cpp | 已有筛选和拟合基础；柱型误差、共同高程关联完善、三维轨道框架与质量标定作为拟研究内容 |
| 稀疏地图定位与后端 | interfaces/ros1/src/pole_localization.cpp、modules/pgo/src/sparse_localization_graph.cpp | 已有汇总部分位姿观测与增量融合；坡道下三维弱信息方向保持和长线索引作为拟研究内容 |

当前轨道汇总因子采用水平横向、高程和姿态分量，报告中的三维法平面模型是拟研究方案。现有纯稀疏定位依赖初值，报告将无先验全线搜索列为扩展内容。scripts/replay_gnss_quality.py 的高程趋势回放不能作为已集成 GNSS 优化的验证。

## 文献与实验表述

参考文献通过原始论文、出版商、作者主页或机构库核实，检索截止到 2026-10-12。参考文献中的 DOI / 原始链接可用于继续阅读。RailLoMer 原始条目采用已核实的 2021 年预印本信息；BIEVR-LIO 采用 RSS 2026 正式会议条目。

报告讨论了与支柱、轨道联合铁路定位直接相关的 Wortmann 等 2025 年工作，因此未将“支柱与轨道组合”或“定向高度图”直接写成本课题首创。拟创新点聚焦于铁路几何适配、参数化提取质量及三维互补定位中的明确改进，仍需实验确认。

未将同一数据建图再复放的误差、GNSS 拟合残差或未经核验的接收机轨迹写成独立真值精度。正式实验需要补充独立重复采集、参考质量说明与公平资源对照。
