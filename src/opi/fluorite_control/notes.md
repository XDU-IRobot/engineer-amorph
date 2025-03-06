# 存矿机构逻辑

- 真空泵${P}$：${\overline{P}}$吸， ${\underline{P}}$吹， ${P}$停

- 左右电磁阀${LR}$：${\overline{L}}$封闭，${L}$连接到真空泵

- 左右存矿机构状态${O/SS/BB/SB/BS}$：${S}$吸，${B}$吹，${O}$停

**设某个电磁阀封闭时，真空泵的状态改变不会影响到该存矿机构的状态**

状态转移表：

| 后一状态\前一状态 | ${O}$                         | ${SS}$                         | ${SB}$              | ${BB}$                        | ${BS}$              |
| ----------------- | ----------------------------- | ------------------------------ | ------------------- | ----------------------------- | ------------------- |
| ${O}$             | -                             | ${PLR}$                        | ${PLR}$             | ${PLR}$                       | ${PLR}$             |
| ${SS}$            | ${\overline{P}LR}$            | -                              | ${\overline{P}LR}$  | ${\overline{P}LR}$            | ${\overline{P}LR}$  |
| ${SB}$            | ${\overline{P}L\overline{R}}$ | ${\underline{P}\overline{L}R}$ | -                   | ${\overline{P}L\overline{R}}$ | via ${SS}$          |
| ${BB}$            | ${\underline{P}LR}$           | ${\underline{P}LR}$            | ${\underline{P}LR}$ | -                             | ${\underline{P}LR}$ |
| ${BS}$            | ${\overline{PL}R}$            | ${\underline{P}L\overline{R}}$ | via ${SS}$          | ${\overline{PL}R}$            | -                   |
