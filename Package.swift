// swift-tools-version:5.9
import PackageDescription

let package = Package(
    name: "swift-tsdf",
    platforms: [
        .macOS(.v12),
        .iOS(.v15),
    ],
    products: [
        .library(name: "TSDF", targets: ["TSDF"]),
    ],
    targets: [
        .target(name: "TSDF"),
        .testTarget(name: "TSDFTests", dependencies: ["TSDF"]),
    ]
)
