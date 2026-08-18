//
//  TSDFVolumeTests.swift
//  swift-tsdf tests
//
//  Plain `import TSDF` (no @testable) on purpose: these tests double as
//  proof that the public API surface is sufficient to drive the library.
//

import XCTest
import simd
import TSDF

final class TSDFVolumeTests: XCTestCase {

    private func makeFrame() -> SyntheticFrame {
        let pose = SyntheticDepth.makeLookAt(
            eye: SIMD3<Float>(0, 0, 1.6),
            target: SIMD3<Float>(0, 0, 0)
        )
        return SyntheticDepth.renderSphereDepth(
            sphereCenter: SIMD3<Float>(0, 0, 0),
            sphereRadius: 0.5,
            width: 128, height: 96,
            fx: 110, fy: 110, cx: 64, cy: 48,
            camToWorld: pose
        )
    }

    private func makeVolume(dim: Int = 64, voxelSize: Float = 0.02) -> TSDFVolume {
        var config = VolumeConfig()
        config.voxelSize = voxelSize
        config.truncation = 4 * voxelSize
        let half = Float(dim) * voxelSize / 2
        return TSDFVolume(
            dim: dim,
            origin: SIMD3<Float>(-half, -half, -half),
            config: config
        )
    }

    func testIntegrateStatsAccounting() {
        let frame = makeFrame()
        let volume = makeVolume()

        var stats = IntegrateStats()
        frame.depth.withUnsafeBufferPointer { d in
            frame.confidence.withUnsafeBufferPointer { c in
                frame.rgb.withUnsafeBufferPointer { r in
                    stats = volume.integrate(
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

        XCTAssertGreaterThan(stats.projected, 0, "no voxels projected into the frame at all")
        XCTAssertGreaterThan(stats.voxelUpdates, 0, "no voxels were written")
        // Every pixel in the synthetic frame carries confidence 2.
        XCTAssertEqual(stats.conf0, 0)
        XCTAssertEqual(stats.conf1, 0)
        XCTAssertEqual(stats.conf2, stats.projected)
        XCTAssertEqual(stats.rejectedByConf, 0)
        // Missed rays write depth 0, which fails the depth-range gate: the
        // volume contains background voxels that project onto those pixels.
        XCTAssertGreaterThan(stats.rejectedByDepthRange, 0)
        // Buckets must account for every projected voxel.
        let accounted = stats.rejectedByConf
            + stats.rejectedByDepthRange
            + stats.rejectedBySDF
            + stats.voxelUpdates
        XCTAssertEqual(accounted, stats.projected, "stats buckets do not sum to projected")
    }

    func testWeightCapAndTSDFRange() {
        let frame = makeFrame()
        let volume = makeVolume()

        frame.depth.withUnsafeBufferPointer { d in
            frame.confidence.withUnsafeBufferPointer { c in
                frame.rgb.withUnsafeBufferPointer { r in
                    for _ in 0..<80 {
                        volume.integrate(
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

        var maxWeight: Float = 0
        var updatedVoxels = 0
        for vox in volume.voxels {
            maxWeight = max(maxWeight, vox.weight)
            if vox.weight > 0 {
                updatedVoxels += 1
                XCTAssertGreaterThanOrEqual(vox.tsdf, -1.0001)
                XCTAssertLessThanOrEqual(vox.tsdf, 1.0001)
            }
        }
        XCTAssertGreaterThan(updatedVoxels, 0)
        // conf-2 frames add weight 2 per update; 80 passes would reach 160
        // uncapped. The running average must clamp at the configured max.
        XCTAssertLessThanOrEqual(maxWeight, 64.0001)
        XCTAssertGreaterThan(maxWeight, 32, "cap never approached; integration too sparse")
    }
}
