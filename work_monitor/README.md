# 数采工时监控

工时账本记录 A/B/C GUI 中每次真正开始的录制 attempt。开始被相机、NAS 或机器人状态
拒绝时不计入；保存、Failure、丢弃、自动核验失败、保存失败和异常退出都会保留记录。

默认数据库：

```text
~/.local/share/frankateleop/work_monitor/worktime.sqlite3
```

SQLite 使用 WAL。写入发生在独立 worker 中；数据库暂时不可写或队列满时，事件会落到同
目录的 `worktime.emergency.jsonl`，后续启动自动尝试回放。监控故障不会阻塞录制线程。

统计口径：

- 实际录制：各 attempt 中真正处于 recording 的 segment 之和，暂停时间不计。
- 有效工作：录制 segment 合并后，相邻间隔 `<=60 秒` 的完整间隔也计入工作。
- 休息：每日首个 attempt 起 9 小时窗口内，未计入有效工作的已流逝时间。
- 全部模式：A/B/C 的时间区间取并集，重叠时间不会重复累计。
- 异常退出：每秒 heartbeat；下次读取账本时按最后 heartbeat 截断并标记 interrupted。

启动：

```bash
bash M_run_worktime_monitor.sh
```

页面支持按人员、task、日期和 A/B/C 模式筛选，并显示最近 7 天时间轴和 attempt 明细。
它不扫描 NAS，也不导入功能上线前的历史 episode。
