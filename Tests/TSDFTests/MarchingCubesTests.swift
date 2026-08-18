//
//  MarchingCubesTests.swift
//  swift-tsdf tests
//
//  Drives the extractor with analytic SDF fills (no camera, no integration)
//  so failures here isolate the marching-cubes stage.
//

import XCTest
import simd
import TSDF

final class MarchingCubesTests: XCTestCase {

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

    /// Fill the volume with an exact truncated sphere SDF.
    private func fillSphere(_ volume: TSDFVolume, center: SIMD3<Float>, radius: Float) {
        let trunc = volume.truncation
        for z in 0..<volume.dim {
            for y in 0..<volume.dim {
                for x in 0..<volume.dim {
                    let p = volume.worldOfVoxel(x, y, z)
                    let d = simd_length(p - center) - radius
                    var t = d / trunc
                    if t > 1 { t = 1 }
                    if t < -1 { t = -1 }
                    var vox = Voxel()
                    vox.tsdf = t
                    vox.weight = 1
                    vox.color = SIMD3<UInt8>(200, 100, 50)
                    volume.voxels[volume.index(x, y, z)] = vox
                }
            }
        }
    }

    func testSphereSDFExtraction() {
        let voxelSize: Float = 0.02
        let radius: Float = 0.4
        let volume = makeVolume(dim: 64, voxelSize: voxelSize)
        fillSphere(volume, center: .zero, radius: radius)

        let mesh = MarchingCubes.extract(from: volume)

        XCTAssertGreaterThan(mesh.triangles.count, 1000,
                             "sphere of this size should tessellate into thousands of triangles")
        XCTAssertEqual(mesh.positions.count, mesh.triangles.count * 3,
                       "unwelded output: exactly 3 vertices per triangle")
        XCTAssertEqual(mesh.colors.count, mesh.positions.count)

        // Sub-voxel interpolation against an exact SDF should place every
        // vertex well within half a voxel of the true surface.
        var maxErr: Float = 0
        for p in mesh.positions {
            let err = abs(simd_length(p) - radius)
            maxErr = max(maxErr, err)
        }
        XCTAssertLessThan(maxErr, 0.5 * voxelSize,
                          "max radial error \(maxErr) exceeds half a voxel")
    }

    func testEmptyVolumeProducesNothing() {
        let volume = makeVolume(dim: 32)
        let mesh = MarchingCubes.extract(from: volume)
        XCTAssertEqual(mesh.triangles.count, 0,
                       "weight-0 voxels are unobserved and must emit no geometry")
    }

    func testUniformPositiveProducesNothing() {
        let volume = makeVolume(dim: 32)
        for i in 0..<volume.voxels.count {
            var vox = Voxel()
            vox.tsdf = 1
            vox.weight = 1
            volume.voxels[i] = vox
        }
        let mesh = MarchingCubes.extract(from: volume)
        XCTAssertEqual(mesh.triangles.count, 0,
                       "a field with no sign change has no surface")
    }
}
