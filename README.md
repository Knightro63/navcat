# navcat

[![Pub Version](https://img.shields.io/pub/v/navcat)](https://pub.dev/packages/navcat)
[![License: MIT](https://img.shields.io/badge/license-MIT-purple.svg)](https://opensource.org/licenses/MIT)

navcat is a dart navigation mesh based on [isaac-mason's](https://github.com/isaac-mason) [navcat](https://github.com/isaac-mason/navcat) to construction and querying library for 3D floor-based navigation.

navcat is ideal for use in games, simulations, and creative websites that require navigation in complex 3D environments.

**Features**

![Image of a 3d model with a navmesh overlay.](https://raw.githubusercontent.com/Knightro63/navcat/master/examples/assets/screenshots/solo_navmesh.png)

- Navigation mesh generation from 3D geometry
- Navigation mesh querying
- Single and multi-tile navigation mesh support
- Fully JSON serializable data structures

## What is a Navigation Mesh?

A navigation mesh (or navmesh) is a simplified representation of a 3D environment that is used for pathfinding and AI navigation in video games and simulations. It consists of interconnected polygons that define walkable areas within the environment. These polygons are connected by edges and off-mesh connections, allowing agents (characters) to move from one polygon to another.

## Example

Find the example for this API [here](https://github.com/Knightro63/navcat/tree/main/example/), for more examples you can click [here](https://github.com/Knightro63/navcat/tree/main/examples/), and for a preview go [here](https://knightro63.github.io/navcat/).

## Contributing

Contributions are welcome.
In case of any problems look at [existing issues](https://github.com/Knightro63/navcat/issues), if you cannot find anything related to your problem then open an issue.
Create an issue before opening a [pull request](https://github.com/Knightro63/navcat/pulls) for non trivial fixes.
In case of trivial fixes open a [pull request](https://github.com/Knightro63/navcat/pulls) directly.

## Acknowledgements

- This library is a ts to dart conversion of [isaac-mason's](https://github.com/isaac-mason) [navcat](https://github.com/isaac-mason/navcat).