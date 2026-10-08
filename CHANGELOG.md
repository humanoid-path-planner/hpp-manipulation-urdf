# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

## [Unreleased]

## [9.0.2] - 2026-07-24



## [9.0.0] - 2026-07-13



## [7.0.0] - 2026-03-06



## [6.1.0] - 2025-10-23



## [6.0.0] - 2024-12-07

Changes in v6.0.0
- hpp-fcl dependency has been replaced by coal
- updates for coal v3


## [5.2.0] - 2024-10-09

Changes in v5.2.0:
- fix deprecated C++11 code
- nix: move package to nixpkgs
- ci: use https
- setup mergify


## [5.1.0] - 2024-07-02

Changes in v5.1.0:
- Add a test on robot appending
- Nix: initial support
- update tooling


## [5.0.0] - 2024-03-31

Changes in v5.0.0:
- :warning: switch to TinyXML2
- update packaging
- update tooling


## [4.15.1] - 2023-01-20



## [4.14.0] - 2022-11-02



## [4.13.0] - 2022-05-31



## [4.12.0] - 2021-10-06



## [4.11.0] - 2021-05-04

Changes in v4.11.0:
- Check that frame does not exist before creating a gripper.
- Reduce dependency to boost.
- Don't silently return on error.

## [4.10.1] - 2020-09-24

Changes since v4.9.0:
* Add package.xml

## [4.10.0] - 2020-08-17



## [4.9.0] - 2020-04-29

Changes in v4.9.0:
- Overload hpp::manipulation::srdf::loadModelFromFile
- CMake Exports

## [4.8.0] - 2019-11-28

Changes since v4.5.0
- Update CMake


## [4.5.0] - 2019-04-25

Change since v4.4.0:
- [CI] include conf from rainboard
- s/BOOST_MESSAGE/BOOST_TEST_MESSAGE


## [4.4.0] - 2019-03-19

Changes since v4.3.0:
- Enhance position tag.
- Update to pinocchio v2


## [4.3.0] - 2019-01-31

- [CI] add .gitlab-ci.yml & badges


## [4.2.0] - 2018-10-11

Changes since v4.1:
- Update device data after adding a frame.
- Update to changes in pinocchio (tools renamed into utils)
- Do not use deprecated class AxialHandle anymore.

## [4.0] - 2018-03-22

From this version on, hpp now depends on pinocchio package for all computations of forward kinematics.

[Unreleased]: https://github.com/humanoid-path-planner/hpp-manipulation-urdf/compare/v9.0.2...HEAD
[9.0.2]: https://github.com/humanoid-path-planner/hpp-manipulation-urdf/compare/v9.0.0...v9.0.2
[9.0.0]: https://github.com/humanoid-path-planner/hpp-manipulation-urdf/compare/v7.0.0...v9.0.0
[7.0.0]: https://github.com/humanoid-path-planner/hpp-manipulation-urdf/compare/v6.1.0...v7.0.0
[6.1.0]: https://github.com/humanoid-path-planner/hpp-manipulation-urdf/compare/v6.0.0...v6.1.0
[6.0.0]: https://github.com/humanoid-path-planner/hpp-manipulation-urdf/compare/v5.2.0...v6.0.0
[5.2.0]: https://github.com/humanoid-path-planner/hpp-manipulation-urdf/compare/v5.1.0...v5.2.0
[5.1.0]: https://github.com/humanoid-path-planner/hpp-manipulation-urdf/compare/v5.0.0...v5.1.0
[5.0.0]: https://github.com/humanoid-path-planner/hpp-manipulation-urdf/compare/v4.15.1...v5.0.0
[4.15.1]: https://github.com/humanoid-path-planner/hpp-manipulation-urdf/compare/v4.14.0...v4.15.1
[4.14.0]: https://github.com/humanoid-path-planner/hpp-manipulation-urdf/compare/v4.13.0...v4.14.0
[4.13.0]: https://github.com/humanoid-path-planner/hpp-manipulation-urdf/compare/v4.12.0...v4.13.0
[4.12.0]: https://github.com/humanoid-path-planner/hpp-manipulation-urdf/compare/v4.11.0...v4.12.0
[4.11.0]: https://github.com/humanoid-path-planner/hpp-manipulation-urdf/compare/v4.10.1...v4.11.0
[4.10.1]: https://github.com/humanoid-path-planner/hpp-manipulation-urdf/compare/v4.10.0...v4.10.1
[4.10.0]: https://github.com/humanoid-path-planner/hpp-manipulation-urdf/compare/v4.9.0...v4.10.0
[4.9.0]: https://github.com/humanoid-path-planner/hpp-manipulation-urdf/compare/v4.8.0...v4.9.0
[4.8.0]: https://github.com/humanoid-path-planner/hpp-manipulation-urdf/compare/v4.5.0...v4.8.0
[4.5.0]: https://github.com/humanoid-path-planner/hpp-manipulation-urdf/compare/v4.4.0...v4.5.0
[4.4.0]: https://github.com/humanoid-path-planner/hpp-manipulation-urdf/compare/v4.3.0...v4.4.0
[4.3.0]: https://github.com/humanoid-path-planner/hpp-manipulation-urdf/compare/v4.2.0...v4.3.0
[4.2.0]: https://github.com/humanoid-path-planner/hpp-manipulation-urdf/compare/v4.0...v4.2.0
[4.0]: https://github.com/humanoid-path-planner/hpp-manipulation-urdf/releases/tag/v4.0
