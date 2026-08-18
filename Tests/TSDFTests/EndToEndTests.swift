//
//  EndToEndTests.swift
//  swift-tsdf tests
//
//  The full pipeline against synthetic ground truth: orbit 16 virtual depth
//  cameras around an analytic sphere, integrate every frame, extract the
//  mesh, and measure reconstruction error against the known geometry. Then
//  round-trip the mesh through the binary PLY writer and verify the byte
//  layout from the header alone.
//

import XCTest
import Foundation
import simd
import TSDF

final class EndToEndTests: XCTestCase {

    private let sphereRadius: Float = 0.5
    private let voxelSize: Float = 0.02

    private func reconstructSphere() -> MarchingCubes.Mesh {
        let dim = 96
        var config = VolumeConfig()
        config.voxelSize = voxelSize
        config.truncation = 4 * voxelSize
        let half = Float(dim) * voxelSize / 2
        let volume = TSDFVolume(
            dim: dim,
            origin: SIMD3<Float>(-half, -half, -half),
            config: config
        )

        var total = IntegrateStats()
        let frames = 16
        for k in 0..<frames {
            let theta = Float(k) / Float(frames) * 2 * Float.pi
            let eye = SIMD3<Float>(1.6 * sin(theta), 0, 1.6 * cos(theta))
            let pose = SyntheticDepth.makeLookAt(eye: eye, target: .zero)
            let frame = SyntheticDepth.renderSphereDepth(
                sphereCenter: .zero,
                sphereRadius: sphereRadius,
                width: 128, height: 96,
                fx: 110, fy: 110, cx: 64, cy: 48,
                camToWorld: pose
            )
            frame.depth.withUnsafeBufferPointer { d in
                frame.confidence.withUnsafeBufferPointer { c in
                    frame.rgb.withUnsafeBufferPointer { r in
                        total = total + volume.integrate(
                            depth: d.baseAddress!,
                            confidence: c.baseAddress!,
                            depthWidth: frame.width,
                            depthHeight: frame.height,
                            rgb: r.baseAddress!,
                            rgbWidth: frame.width,
                            rgbHeight: frame.height,
                            depthIntrinsics: frame.intrinsics,
                            camToWorld: frame.camToWorld
                        )
                    }
                }
            }
        }
        XCTAssertGreaterThan(total.voxelUpdates, 0, "orbit integration wrote nothing")

        return MarchingCubes.extract(from: volume)
    }

    func testOrbitSphereReconstruction() {
        let mesh = reconstructSphere()

        XCTAssertGreaterThan(mesh.triangles.count, 500,
                             "reconstruction produced too little geometry")

        var sumErr: Float = 0
        var maxErr: Float = 0
        for p in mesh.positions {
            let err = abs(simd_length(p) - sphereRadius)
            sumErr += err
            maxErr = max(maxErr, err)
        }
        let meanErr = sumErr / Float(mesh.positions.count)

        XCTAssertLessThan(meanErr, voxelSize,
                          "mean radial error \(meanErr) exceeds one voxel")
        XCTAssertLessThan(maxErr, 2.5 * voxelSize,
                          "max radial error \(maxErr) exceeds 2.5 voxels")
    }

    func testPLYMeshRoundTrip() throws {
        let mesh = reconstructSphere()
        let url = FileManager.default.temporaryDirectory
            .appendingPathComponent("swift-tsdf-test-\(UUID().uuidString).ply")
        defer { try? FileManager.default.removeItem(at: url) }

        try PLYBinaryExporter.writeMesh(
            positions: mesh.positions,
            colors: mesh.colors,
            triangles: mesh.triangles,
            to: url
        )

        let data = try Data(contentsOf: url)
        let marker = Data("end_header\n".utf8)
        guard let range = data.range(of: marker) else {
            XCTFail("PLY header terminator not found")
            return
        }
        let headerLen = range.upperBound
        let header = String(decoding: data[..<headerLen], as: UTF8.self)

        XCTAssertTrue(header.hasPrefix("ply\n"), "not a PLY file")
        XCTAssertTrue(header.contains("format binary_little_endian 1.0"))
        XCTAssertTrue(header.contains("element vertex \(mesh.positions.count)"))
        XCTAssertTrue(header.contains("element face \(mesh.triangles.count)"))

        // 15 bytes per vertex (3 float32 + 3 uchar),
        // 13 bytes per face (1 uchar count + 3 int32 indices).
        let expected = headerLen
            + mesh.positions.count * 15
            + mesh.triangles.count * 13
        XCTAssertEqual(data.count, expected,
                       "binary payload size does not match header-declared counts")
    }
}
