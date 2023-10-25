use bevy::{prelude::{Component, Bundle, Visibility, ComputedVisibility}, transform::TransformBundle, utils::Uuid};

#[derive(Component)]
pub struct StickmanBody;

#[derive(Component)]
pub struct StickmanBodyPart {
    /// Radius of capsule
    pub radius: f32,
    /// Depth of capsule
    pub depth: f32,
}

impl StickmanBodyPart {
    #[inline(always)]
    pub fn new(radius: f32, depth: f32) -> Self {
        Self { radius, depth }
    }

    #[inline(always)]
    pub fn length(&self) -> f32 {
        self.depth + self.radius
    }
}

//might add more onto this component
#[derive(Component, Clone, Copy)]
pub struct StickmanArmSegment {
    arm_uuid: Uuid,
    segment_len: f32,
    joint_gap: f32,
}

impl StickmanArmSegment {
    #[inline(always)]
    pub fn new(arm_uuid: Uuid, segment_len: f32, joint_gap: f32, radius: f32, motor_params: ArmMotorParams) -> Self {
        

        Self {
            arm_uuid,
            segment_len,
            joint_gap,
        }
    }

    #[inline(always)]
    pub fn get_arm_uuid(&self) -> Uuid {
        self.arm_uuid
    }

    #[inline(always)]
    pub fn get_segment_len(&self) -> f32 {
        self.segment_len
    }

    #[inline(always)]
    pub fn get_joint_gap_len(&self) -> f32 {
        self.joint_gap
    }

    #[inline(always)]
    pub fn get_arm_len(&self) -> f32 {
        self.segment_len + self.joint_gap
    }

}

pub struct ArmMotorParams {
    pub target_pos: f32,
    pub target_vel: f32,
    pub stiffness: f32,
    pub damping: f32,
}

#[derive(Default, Bundle)]
pub struct StickmanMeshParentBundle {
    pub visiblity: Visibility,
    pub computed_visibility: ComputedVisibility,
    pub transform_bundle: TransformBundle,
}