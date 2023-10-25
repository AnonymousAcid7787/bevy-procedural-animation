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
pub struct StickmanArmSegment (Uuid);

impl StickmanArmSegment {
    #[inline(always)]
    pub fn new(arm_uuid: Uuid) -> Self{
        Self(arm_uuid)
    }
}

#[derive(Default, Bundle)]
pub struct StickmanMeshParentBundle {
    pub visiblity: Visibility,
    pub computed_visibility: ComputedVisibility,
    pub transform_bundle: TransformBundle,
}