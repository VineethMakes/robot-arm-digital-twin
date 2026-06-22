use serde::{Deserialize, Serialize};

const DEG: f64 = std::f64::consts::PI / 180.0;
const RAD: f64 = 180.0 / std::f64::consts::PI;

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub fn distance(self, other: Self) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let dz = self.z - other.z;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct JointLimit {
    pub min_deg: f64,
    pub max_deg: f64,
}

impl JointLimit {
    fn clamp(self, value: f64) -> f64 {
        value.max(self.min_deg).min(self.max_deg)
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ArmModel {
    pub base_height_m: f64,
    pub upper_arm_m: f64,
    pub forearm_m: f64,
    pub wrist_m: f64,
    pub limits: [JointLimit; 4],
}

impl Default for ArmModel {
    fn default() -> Self {
        Self {
            base_height_m: 0.16,
            upper_arm_m: 0.28,
            forearm_m: 0.24,
            wrist_m: 0.12,
            limits: [
                JointLimit {
                    min_deg: -170.0,
                    max_deg: 170.0,
                },
                JointLimit {
                    min_deg: -50.0,
                    max_deg: 115.0,
                },
                JointLimit {
                    min_deg: -135.0,
                    max_deg: 135.0,
                },
                JointLimit {
                    min_deg: -120.0,
                    max_deg: 120.0,
                },
            ],
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct JointAngles {
    pub base_yaw_deg: f64,
    pub shoulder_deg: f64,
    pub elbow_deg: f64,
    pub wrist_deg: f64,
}

impl Default for JointAngles {
    fn default() -> Self {
        Self {
            base_yaw_deg: 0.0,
            shoulder_deg: 25.0,
            elbow_deg: 55.0,
            wrist_deg: -25.0,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ArmPose {
    pub joints: JointAngles,
    pub points: Vec<Vec3>,
    pub end_effector: Vec3,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct IkSolution {
    pub pose: ArmPose,
    pub target: Vec3,
    pub error_m: f64,
    pub iterations: usize,
    pub reachable: bool,
}

pub fn default_model() -> ArmModel {
    ArmModel::default()
}

pub fn forward_kinematics(model: &ArmModel, joints: JointAngles) -> ArmPose {
    let yaw = joints.base_yaw_deg * DEG;
    let planar = planar_points(model, joints);
    let points = planar
        .into_iter()
        .map(|(r, y)| Vec3::new(r * yaw.cos(), y, r * yaw.sin()))
        .collect::<Vec<_>>();
    let end_effector = *points.last().expect("arm always has an end effector");

    ArmPose {
        joints,
        points,
        end_effector,
    }
}

pub fn solve_ik(model: &ArmModel, target: Vec3) -> IkSolution {
    let base_yaw_deg = target.z.atan2(target.x) * RAD;
    let radial_target = (target.x * target.x + target.z * target.z).sqrt();
    let y_target = target.y - model.base_height_m;
    let lower_arm_m = model.forearm_m + model.wrist_m;
    let target_distance = radial_target.hypot(y_target);
    let max_reach = model.upper_arm_m + lower_arm_m;
    let min_reach = (model.upper_arm_m - lower_arm_m).abs();
    let solved_distance = target_distance.clamp(min_reach + 0.001, max_reach - 0.001);
    let target_angle = y_target.atan2(radial_target);
    let shoulder_offset = law_of_cosines(solved_distance, model.upper_arm_m, lower_arm_m);
    let elbow_inside = law_of_cosines(model.upper_arm_m, lower_arm_m, solved_distance);

    let shoulder_deg = (target_angle - shoulder_offset) * RAD;
    let elbow_deg = (std::f64::consts::PI - elbow_inside) * RAD;

    let joints = JointAngles {
        base_yaw_deg: model.limits[0].clamp(base_yaw_deg),
        shoulder_deg: model.limits[1].clamp(shoulder_deg),
        elbow_deg: model.limits[2].clamp(elbow_deg),
        wrist_deg: 0.0,
    };
    let pose = forward_kinematics(model, joints);
    let error_m = pose.end_effector.distance(target);

    IkSolution {
        pose,
        target,
        error_m,
        iterations: 1,
        reachable: radial_target.hypot(y_target) <= max_reach + 0.01 && error_m < 0.04,
    }
}

fn planar_points(model: &ArmModel, joints: JointAngles) -> Vec<(f64, f64)> {
    let mut points = vec![(0.0, 0.0)];
    let mut angle = 0.0;
    let mut current = (0.0, 0.0);

    for (joint_deg, length) in [
        (joints.shoulder_deg, model.upper_arm_m),
        (joints.elbow_deg, model.forearm_m),
        (joints.wrist_deg, model.wrist_m),
    ] {
        angle += joint_deg * DEG;
        current.0 += length * angle.cos();
        current.1 += length * angle.sin();
        points.push((current.0, current.1 + model.base_height_m));
    }

    points
}

fn law_of_cosines(adjacent_a: f64, adjacent_b: f64, opposite: f64) -> f64 {
    let numerator = adjacent_a.powi(2) + adjacent_b.powi(2) - opposite.powi(2);
    let denominator = 2.0 * adjacent_a * adjacent_b;
    (numerator / denominator).clamp(-1.0, 1.0).acos()
}

#[cfg(feature = "wasm")]
#[wasm_bindgen::prelude::wasm_bindgen]
pub fn solve_arm(target_x: f64, target_y: f64, target_z: f64) -> String {
    let solution = solve_ik(
        &ArmModel::default(),
        Vec3::new(target_x, target_y, target_z),
    );
    serde_json::to_string(&solution).expect("solution should serialize")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_pose_has_four_points() {
        let pose = forward_kinematics(&ArmModel::default(), JointAngles::default());
        assert_eq!(pose.points.len(), 4);
    }

    #[test]
    fn ik_gets_close_to_reachable_target() {
        let target = Vec3::new(0.38, 0.28, 0.12);
        let solution = solve_ik(&ArmModel::default(), target);
        assert!(solution.error_m < 0.06, "error was {}", solution.error_m);
    }

    #[test]
    fn yaw_tracks_target_side() {
        let target = Vec3::new(0.25, 0.22, 0.25);
        let solution = solve_ik(&ArmModel::default(), target);
        assert!(solution.pose.joints.base_yaw_deg > 30.0);
    }
}
