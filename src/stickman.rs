use bevy::{prelude::{Component, Bundle, Visibility, ComputedVisibility, Entity}, transform::TransformBundle, utils::Uuid};

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
#[derive(Component)]
pub struct StickmanArmSegment {
    arm_uuid: Uuid,
    /// If this is not at the end of an arm's heirarchy, this would have a value containing the child arm.
    /// This might become a Vec of entities later on, to support multiple arm segments attached to one upper arm.
    lower_arm: Option<Entity>,
}

impl StickmanArmSegment {
    #[inline(always)]
    pub fn new(arm_uuid: Uuid, lower_arm: Option<Entity>) -> Self {
        Self {
            arm_uuid,
            lower_arm
        }
    }

    #[inline(always)]
    pub fn is_lowest_arm(&self) -> bool {
        self.lower_arm.is_none()
    }

    #[inline(always)]
    pub fn arm_uuid(&self) -> Uuid {
        self.arm_uuid
    }
}

#[derive(Default, Bundle)]
pub struct StickmanMeshParentBundle {
    pub visiblity: Visibility,
    pub computed_visibility: ComputedVisibility,
    pub transform_bundle: TransformBundle,
}