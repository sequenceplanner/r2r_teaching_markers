# r2r_teaching_markers

This package provides an interactive marker server implemented in Rust for live transform manipulation with visualization. It allows users to create and manipulate interactive markers with controls for rotation and translation along all three axes.

## Features

- **Interactive Markers**: Create custom interactive markers that can be moved and rotated along the X, Y, and Z axes.
- **Real-Time Feedback**: Handles feedback from the interactive markers to update transforms in real-time.
- **Transform Publishing**: Publish transforms to the `/tf_static` topic, enabling dynamic visualization in tools like RViz.
- **Threading Support**: Utilize multi-threading and channels for asynchronous communication and efficient data handling.

## Overview

The `TeachingMarker` struct represents a teaching marker within the interactive marker server. By using this package, developers can enhance robot teaching interfaces by allowing users to intuitively manipulate markers in a 3D space. This is particularly useful for applications like setting up frames, poses, or tools in robotic systems.

## Getting Started

### Prerequisites

- Rust programming language
- ROS2 environment
- [`r2r`](https://crates.io/crates/r2r) crate for ROS2 communication
- [`tokio`](https://crates.io/crates/tokio) for asynchronous operations

### Installation

Add the following to your `Cargo.toml`:

```toml
[dependencies]
r2r = "0.9.0"
r2r_teaching_markers = { git = "https://github.com/sequenceplanner/r2r_teaching_markers", tag = "v0.0.1" }
tokio = { version = "1.36.0", features = ["full"] }

```