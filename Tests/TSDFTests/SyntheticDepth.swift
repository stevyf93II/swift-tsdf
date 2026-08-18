//
//  SyntheticDepth.swift
//  swift-tsdf tests
//
//  Analytic depth-frame renderer used by the integration tests. Renders a
//  perfect sphere with per-pixel ray casting under the exact same camera
//  convention the integrator assumes:
//
//    Camera frame: +X right, +Y up, -Z forward.
//    Unproject (u, v, d) -> ((u-cx)*d/fx, -(v-cy)*d/fy, -d).
//
//  The per-pixel ray direction is deliberately UNNORMALIZED:
//  ((u-cx)/fx, -(v-cy)/fy, -1), so the ray parameter t IS the z-depth.
//  Missed pixels write depth 0, which the integrator rejects via the
//  configured depth range lower bound.
//

import Foundation
import simd

struct SyntheticFrame {
    var depth: [Float]
    var confidence: [UInt8]
    var rgb: [UInt8]          // 4 bytes per pixel, same resolution as depth
    var width: Int
    var height: Int
    var intrinsics: simd_float3x3
    var camToWorld: simd_float4x4
}

enum SyntheticDepth {

    /// Right-handed look-at producing a camToWorld with the -Z-forward
    /// convention: columns are (right, up, -forward, eye).
    static func makeLookAt(
        eye: SIMD3<Float>,
        target: SIMD3<Float>,
        up: SIMD3<Float> = SIMD3<Float>(0, 1, 0)
    ) -> simd_float4x4 {
        let f = simd_normalize(target - eye)          // forward
        let s = simd_normalize(simd_cross(f, up))     // right
        let u = simd_cross(s, f)                      // true up
        let z = -f                                    // camera +Z is backward
        return simd_float4x4(
            SIMD4<Float>(s.x, s.y, s.z, 0),
            SIMD4<Float>(u.x, u.y, u.z, 0),
            SIMD4<Float>(z.x, z.y, z.z, 0),
            SIMD4<Float>(eye.x, eye.y, eye.z, 1)
        )
    }

    /// Ray-cast a sphere into a depth frame from the given pose.
    /// Every hit pixel gets confidence 2 (high); every pixel gets a constant
    /// gray RGB so color averaging stays deterministic.
    static func renderSphereDepth(
        sphereCenter: SIMD3<Float>,
        sphereRadius: Float,
        width: Int,
        height: Int,
        fx: Float, fy: Float, cx: Float, cy: Float,
        camToWorld: simd_float4x4
    ) -> SyntheticFrame {
        var depth = [Float](repeating: 0, count: width * height)
        let confidence = [UInt8](repeating: 2, count: width * height)
        var rgb = [UInt8](repeating: 0, count: width * height * 4)
        for i in 0..<(width * height) {
            rgb[i * 4 + 0] = 128
            rgb[i * 4 + 1] = 128
            rgb[i * 4 + 2] = 128
            rgb[i * 4 + 3] = 255
        }

        let eye = SIMD3<Float>(camToWorld[3][0], camToWorld[3][1], camToWorld[3][2])
        let rot = simd_float3x3(
            SIMD3<Float>(camToWorld[0][0], camToWorld[0][1], camToWorld[0][2]),
            SIMD3<Float>(camToWorld[1][0], camToWorld[1][1], camToWorld[1][2]),
            SIMD3<Float>(camToWorld[2][0], camToWorld[2][1], camToWorld[2][2])
        )

        let oc = eye - sphereCenter
        let cq = simd_dot(oc, oc) - sphereRadius * sphereRadius

        for v in 0..<height {
            for u in 0..<width {
                // Unnormalized camera-space ray: t along this ray equals z-depth.
                let dirCam = SIMD3<Float>(
                    (Float(u) - cx) / fx,
                    -(Float(v) - cy) / fy,
                    -1
                )
                let dir = rot * dirCam
                let a = simd_dot(dir, dir)
                let b = 2 * simd_dot(oc, dir)
                let disc = b * b - 4 * a * cq
                if disc < 0 { continue }
                let sq = disc.squareRoot()
                var t = (-b - sq) / (2 * a)
                if t <= 0 { t = (-b + sq) / (2 * a) }
                if t <= 0 { continue }
                depth[v * width + u] = t
            }
        }

        let intr = simd_float3x3(
            SIMD3<Float>(fx, 0, 0),
            SIMD3<Float>(0, fy, 0),
            SIMD3<Float>(cx, cy, 1)
        )

        return SyntheticFrame(
            depth: depth,
            confidence: confidence,
            rgb: rgb,
            width: width,
            height: height,
            intrinsics: intr,
            camToWorld: camToWorld
        )
    }
}
