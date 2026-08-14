# navcat

[![Pub Version](https://img.shields.io/pub/v/navcat)](https://pub.dev/packages/navcat)
[![analysis](https://github.com/Knightro63/navcat/actions/workflows/flutter.yml/badge.svg)](https://github.com/Knightro63//navcat/actions/)
[![License: MIT](https://img.shields.io/badge/license-MIT-purple.svg)](https://opensource.org/licenses/MIT)

navcat is a dart navigation mesh based on [navcat](https://github.com/isaac-mason/navcat) to construction and querying library for 3D floor-based navigation.

navcat is ideal for use in games, simulations, and creative websites that require navigation in complex 3D environments.

**Features**

- Navigation mesh generation from 3D geometry
- Navigation mesh querying
- Single and multi-tile navigation mesh support
- Fully JSON serializable data structures

## What is a Navigation Mesh?

A navigation mesh (or navmesh) is a simplified representation of a 3D environment that is used for pathfinding and AI navigation in video games and simulations. It consists of interconnected polygons that define walkable areas within the environment. These polygons are connected by edges and off-mesh connections, allowing agents (characters) to move from one polygon to another.

## Contributing

Contributions are welcome.
In case of any problems look at [existing issues](https://github.com/Knightro63/three_js/issues), if you cannot find anything related to your problem then open an issue.
Create an issue before opening a [pull request](https://github.com/Knightro63/three_js/pulls) for non trivial fixes.
In case of trivial fixes open a [pull request](https://github.com/Knightro63/three_js/pulls) directly.

## Acknowledgements

- This library is a ts to dart conversion of [navcat](https://github.com/isaac-mason/navcat).