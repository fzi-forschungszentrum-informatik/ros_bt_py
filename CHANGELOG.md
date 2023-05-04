# Changelog

History of changes and bugfixes for ros_bt_py

## [Unsynced]


## [v1.1.0 - Dev Sync 08-05-2023]

### Added

- Tick Frequency Topic can now be used to monitor BT Performance
- General Service and Action Nodes are added. Use them to make Calls more precise and get your
  desired outputs directly from the Caller Node
- Capabilities are now added! New and Improved way to distribute Tasks
- Versions! To better keep track on development and in preparation for our ROS 2 release at some
  point in the future we introduced version tags - starting from v1.0.0, which is before
  capabilities are introduced with this sync

### Changed

- Dev Sync Merge Template was improved
- Wait Node time to wait is now float instead of int

### Fixed

- Debug Output for Action Server Success was removed
- Some Pre-Commit Style Issues


## [v1.0.0 - Dev Sync 08-02-2023]

### Added

- ROS Diagnostics - Information about the current status of the BT (ticking, not ticking, error)
- Templates - New templates for common issue types and sync merges

### Changed

- Async Service Proxy - Now utilizes a singleton architecture utilizing threads instead of processes

### Fixed

- Action Termination - Actions now properly wait for a result before the Node returns success
- Code Coverage - Reenabled code coverage to be extracted from ci


## [Dev Sync 20-01-2023]

### Added

- Pre-commit config - Formatting and linting, also protects master branch from direct commits
- NoInputParam Node - Node can get ROSParams from the parameter server without wired inputs
- CHANGELOG.md - File to keep track of changes in the project
- Dual Tree Arguments - Launching in dual mode now allows for loading two separate trees

### Removed

- Melodic CI support - was e.o.l

### Fixed

- Fixed code styling - Fixed a lot of formatting issues noticed by introduced pre-commit and ci


[Unsynced]: https://ids-git.fzi.de/ros/ros_bt_py/compare/master...dev
[v1.1.0 - Dev Sync 08-05-2023]: https://ids-git.fzi.de/ros/ros_bt_py/compare/6d3e71ba...5ff6975a
[v1.0.0 - Dev Sync 08-02-2023]: https://ids-git.fzi.de/ros/ros_bt_py/compare/ba212432...6d3e71ba
[Dev Sync 20-01-2023]: https://ids-git.fzi.de/ros/ros_bt_py/commits/ba212432

<!---
## [Dev Sync DD-MM-YYYY]

### Added

- Put all Additions to the repository in here

### Changed

- Put all Changes in existing functionality here

### Deprecated

- Put all soon-to-be removed features here

### Removed

- Put all removed features here

### Fixed

- Put bugfixes here

[Dev Sync DD-MM-YYYY]: https://ids-git.fzi.de/ros/ros_bt_py/compare/OLDHASH...NEWHASH
-->
