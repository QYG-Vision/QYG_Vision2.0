# QLF 分支协作约定

## 分支角色

`qlf-clean` 是当前视觉工程模板分支，用来保存能编译、能运行、资源完整的基线状态。

个人日常开发不要直接在 `qlf-clean` 上提交。每个队员从 `qlf-clean` 拉出自己的分支，分支名使用姓名拼音缩写，例如：

```bash
git checkout qlf-clean
git pull --rebase origin qlf-clean
git checkout -b zjh
```

如果同一个队员同时做多个方向，可以在缩写后加任务名：

```bash
git checkout -b zjh-auto-aim
git checkout -b zjh-nav-bridge
```

## 新队员克隆流程

```bash
git clone git@github.com:QYG-Vision/QYG_Vision2.0.git
cd QYG_Vision2.0
git checkout qlf-clean
git lfs pull
git checkout -b <姓名拼音缩写>
```

`git lfs pull` 用来下载模型和运行资源。模型资源由 `qlf-clean` 统一维护，个人分支平时不要改模型文件。

## 日常开发流程

开发前同步模板分支：

```bash
git checkout qlf-clean
git pull --rebase origin qlf-clean
git checkout <姓名拼音缩写>
git rebase qlf-clean
```

提交前检查暂存内容：

```bash
git status
git diff --cached --stat
```

不要无脑使用 `git add .`。优先按文件添加：

```bash
git add src/QYG_sentry_debug.cpp
git add tasks/auto_aim/solver.cpp
```

确认无误后提交并推送个人分支：

```bash
git commit -m "简短说明本次改动"
git push origin <姓名拼音缩写>
```

## 模型文件规则

`assets/*.onnx`、`assets/*.bin`、`assets/*.xml` 和 layout 文件由 Git LFS 管理。模型只在确实更新时提交，普通代码改动不要带模型文件。

如果 `git status` 中出现模型文件变更，先确认是否真的换了模型。没有换模型时，不要把它们加入提交。

## 合并规则

个人分支验证完成后，再把改动合并回 `qlf-clean`。合并前需要确认：

- 能编译相关目标。
- 没有提交 build、install、logs、records、缓存文件。
- 没有无关大文件。
- 模型文件只在确实更新时出现。
- 文档、协议、测试和代码改动尽量分组提交。

`qlf-before-clean-sync` 只作为历史备份，不再作为日常开发模板。
