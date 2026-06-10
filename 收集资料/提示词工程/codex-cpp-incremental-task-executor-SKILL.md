---
name: cpp-incremental-task-executor
description: Use for C++ implementation tasks that need incremental decomposition, per-step validation, bug containment, and clean workspace handoff. Especially relevant for rm_vision_2025 vision, navigation, serial, ROS2, EKF, planner, and gimbal-control changes.
---

# C++ Incremental Task Executor

## Purpose

Use this workflow when modifying C++ code in this repository. The goal is to keep every change small, validated, and reversible: decompose the work, implement one task at a time, verify immediately, and do not carry known bugs into the next task.

## Start Marker

Before beginning implementation, address the user as `队长` once. This is a lightweight confirmation that this workflow is active. Keep the rest of the response natural and concise.

Do not use unusual or performative titles unless the user explicitly asks.

## When To Use

Use this workflow when the user asks for:
- C++ code changes across one or more files.
- RoboMaster vision/navigation/debugging implementation work.
- Multi-step implementation, refactoring, bug fixing, or parameter-tooling changes.
- “拆任务”, “分步执行”, “逐个验证”, “bug 不堆积”, “增量开发”, or similar wording.

For tiny one-line edits, keep the process lightweight but still verify.

## Core Rules

- Decompose first when the task has multiple moving parts.
- Ask only for information that cannot be inferred safely from the repo.
- Prefer existing local patterns over new abstractions.
- Prefer C++ standard library facilities when they fit.
- Introduce third-party dependencies only when the standard library and existing project dependencies are insufficient, and explain why.
- Modify only files needed for the current task.
- Verify each task before moving to the next.
- If verification fails, stop and fix that failure before continuing.
- Never hide a known bug behind later work.
- Preserve user changes already present in the working tree.
- After large vision/navigation project changes, create a local git checkpoint for Codex's own changes: inspect `git status`, stage only intended files, and commit with a concise Chinese message and body describing what changed and how it was verified. Do not include unrelated user edits.
- When the user says their understanding has deepened and asks to write it into the learning log, create or update a Markdown file under `学习日志/` following `学习日志/学习总结格式说明书.md`: write in Chinese, connect the idea to this project, include core concepts, project application, learning gains, next steps, and summary time.
- When a debugging session reaches a useful conclusion and the user asks to record debugging ideas or techniques, create or update a Markdown file under `调试心得/` following `调试心得/调试心得格式说明书.md`: focus on reusable debugging methods, key observations, channels/logs, pitfalls, evidence, and advice for future teammates.

## Workflow

### 1. Orient

Inspect the relevant files, current git status, and existing patterns. For searches, prefer `rg` and `rg --files`.

If the task is non-trivial, summarize:
- What the user wants.
- The smallest independent tasks.
- The verification command or observation for each task.

Do not wait for approval unless the user asks for a plan, the change is risky, or requirements are ambiguous.

### 2. Implement One Task

For each task:
- Make the smallest practical edit.
- Keep debug switches and diagnostics clearly named.
- Keep reusable helpers in appropriate shared locations when reuse is real.
- Avoid unrelated cleanup.

### 3. Verify Immediately

Run the narrowest meaningful verification:
- For CMake targets, prefer building the touched target first, e.g. `cmake --build build --target <target>`.
- Run focused tests or scripts when available.
- For hardware/vision debugging, emit structured data where possible and state what the user should observe if hardware verification is needed.

If verification fails:
- Diagnose the root cause.
- Fix only that failure.
- Re-run verification.
- If blocked, report the blocker clearly.

### 4. Continue Or Stop

Only move to the next task after the current one is verified or explicitly blocked.

When all tasks are done:
- Run `git status --short`.
- Mention modified/added files.
- Report verification actually run.
- Note any remaining hardware/manual checks.

### 5. Local Checkpoint For Large Changes

For large feature additions, debugging-tool upgrades, or multi-file vision/navigation changes:
- Run verification first.
- Review `git status --short` and `git diff`.
- Stage only Codex-created or Codex-modified files that belong to the completed task.
- Create a local commit with a Chinese subject and body, for example:
  `调试: 增加自瞄闭环量化分析工具`
- In the commit body, include the main changes and verification result.

If unrelated user changes are present, leave them unstaged and mention them.

### 6. Learning Log

When the user asks to record a newly deepened understanding:
- Use `学习日志/学习总结格式说明书.md` as the format source.
- Write the log in `学习日志/`.
- Use Chinese.
- Prefer updating an existing same-day/same-topic note when appropriate; otherwise create a new note with the existing naming style.
- Include project-specific examples from the current codebase when useful.

### 7. Debugging Notes

When the user asks to record a solved or partially solved debugging experience:
- Use `调试心得/调试心得格式说明书.md` as the format source.
- Write the note in `调试心得/`.
- Use Chinese.
- Prefer updating an existing same-topic note when appropriate; otherwise create a new note named `YYYY-MM-DD_调试主题.md`.
- Focus on techniques for future teammates: phenomenon, assumptions, isolation order, key VOFA/CSV/OpenCV/log fields, effective tricks, pitfalls, evidence, conclusion, and next advice.
- Keep unverified guesses clearly marked as guesses.

## C++ Standard Library Preference

Prefer:
- Containers: `std::vector`, `std::array`, `std::map`, `std::unordered_map`, `std::deque`.
- Algorithms: `<algorithm>`, `<numeric>`.
- Strings: `std::string`, `std::string_view` when appropriate.
- Time: `<chrono>`.
- Files: `<filesystem>` when C++17 is available.
- Concurrency: `std::thread`, `std::mutex`, `std::atomic`, `std::future`.
- Ownership: `std::unique_ptr`, `std::shared_ptr`, references, and value semantics.

Use existing dependencies already present in the project, such as OpenCV, Eigen, nlohmann/json, ROS2, or local `tools/`, when they are already the project pattern.

## Final Response Shape

Keep the final response short:
- What changed.
- What verification passed or could not be run.
- What the user should do next if hardware/manual validation is required.
