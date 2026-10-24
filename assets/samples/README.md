# 参考样例 (真实装甲板照片)

用于回归验证 YOLO 模型与阈值, **不参与构建**。

| 文件 | 内容 | 实测 max score | 分类 |
| --- | --- | --- | --- |
| `sample_red3.jpg`  | 红色装甲板, 白色数字 "3", 环境光充足 | **0.903** | `Rthree` |
| `sample_blue5.jpg` | 蓝色装甲板, 白色数字 "5", 暗环境靠 LED 自发光 | **0.320** | `Bfive` |

验证命令:

```bash
./build/yolo_test assets/samples/sample_red3.jpg  /tmp/o1.jpg
./build/yolo_test assets/samples/sample_blue5.jpg /tmp/o2.jpg
```

两张都应该 `boxes=1` 且 `corners=4`。

## 这两张样例定下了 conf 阈值

模型分类全都正确 —— 连板上的数字都读对了("3"->Rthree, "5"->Bfive) ——
说明权重和输出布局都没问题。但蓝色那张只有 0.32: 画面暗、对比度低,
LED 自发光是主要光源。而比赛场地上我们的相机看到的正是这种情况,
所以 `YOLO_CONF_THRESH` 定 0.30 而不是 0.4/0.5。

红色那张 0.90, 说明模型本身有足够余量, 0.30 不会引入明显误检。
