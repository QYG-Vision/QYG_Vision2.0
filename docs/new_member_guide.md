# 新队员上手指南

这份文档面向刚加入视觉组、准备基于 `qlf-clean` 开发的队员。

核心原则很简单：

```text
qlf-clean 是模板分支，不是个人开发分支。
每个人都从 qlf-clean 拉出自己的并列分支，再在自己的分支上写代码。
```

## 1. 克隆仓库

```bash
git clone git@github.com:QYG-Vision/QYG_Vision2.0.git
cd QYG_Vision2.0
```

如果没有配置 SSH key，也可以用 HTTPS 地址克隆。

## 2. 切到模板分支

```bash
git checkout qlf-clean
git pull --rebase origin qlf-clean
```

`qlf-clean` 是当前推荐的视觉工程模板分支，包含可运行源码、必要模型资源和调试文档。

## 3. 下载模型资源

模型文件由 Git LFS 管理。第一次使用前执行：

```bash
git lfs install
git lfs pull
```

如果没有安装 Git LFS，先安装：

```bash
sudo apt install git-lfs
```

模型文件只由模板分支统一维护。普通代码开发不要修改或提交模型文件。

## 4. 创建自己的并列分支

不要直接在 `qlf-clean` 上写代码。请用自己的姓名拼音缩写创建分支：

```bash
git checkout -b <姓名拼音缩写>
```

例如：

```bash
git checkout -b zjh
git checkout -b wlc
git checkout -b ljx
```

如果一个人同时做多个任务，可以加任务名：

```bash
git checkout -b zjh-auto-aim
git checkout -b zjh-nav-bridge
```

这些分支和 `qlf-clean` 是并列关系，不会污染模板分支。

## 5. 日常开发前同步模板

每次开始开发前，先更新模板分支，再把自己的分支 rebase 到最新模板上：

```bash
git checkout qlf-clean
git pull --rebase origin qlf-clean
git checkout <姓名拼音缩写>
git rebase qlf-clean
```

如果 rebase 发生冲突，先解决冲突再继续，不要用强制覆盖的方式处理。

## 6. 提交代码

先查看变更：

```bash
git status
```

不要直接无脑 `git add .`。优先按文件添加：

```bash
git add src/xxx.cpp
git add tasks/auto_aim/xxx.cpp
git add docs/xxx.md
```

提交前确认暂存区：

```bash
git diff --cached --stat
```

确认没有混入 `build/`、`install/`、`logs/`、`records/`、缓存文件或误改模型后，再提交：

```bash
git commit -m "简短说明本次改动"
git push origin <姓名拼音缩写>
```

## 7. 合并回模板分支

个人分支验证通过后，再申请把改动合并回 `qlf-clean`。合并前至少确认：

- 相关目标能编译。
- 程序能按预期运行。
- 没有提交生成物、缓存、大视频、日志。
- 没有误提交模型文件。
- 文档、测试和代码改动分组清楚。

完整协作规则见 [branch_workflow.md](branch_workflow.md)。
