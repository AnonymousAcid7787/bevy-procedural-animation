use bevy::prelude::*;
use bevy_rapier3d::prelude::RapierMultibodyJointHandle;
use smallvec::SmallVec;

//might add more onto this component
#[derive(Component)]
pub struct StickmanArmSegment {
    /// The arm segment above this one in the arm heirarchy
    upper_arm: Option<Entity>,
    /// The arm segment below this one in the arm heirarchy
    lower_arm: Option<Entity>,
}

impl StickmanArmSegment {
    #[inline(always)]
    pub fn with_upper_arm(upper_arm: Entity) -> Self {
        Self::new(Some(upper_arm), None)
    }

    #[inline(always)]
    pub fn with_lower_arm(lower_arm: Entity) -> Self {
        Self::new(None, Some(lower_arm))
    }

    #[inline(always)]
    pub fn new(upper_arm: Option<Entity>, lower_arm: Option<Entity>) -> Self {
        Self {
            lower_arm,
            upper_arm
        }
    }

    #[inline(always)]
    pub fn is_lowest_arm(&self) -> bool {
        self.lower_arm.is_none()
    }

    #[inline(always)]
    pub fn is_highest_arm(&self) -> bool {
        self.upper_arm.is_none()
    }
}


#[derive(Component)]
pub struct StickmanBody {
    left_arm: StickmanArm,
    right_arm: StickmanArm,
}

//Stickman arm component (parent of both upper and lower arm(s))
#[derive(Component, Default)]
pub struct StickmanArm {
    pub arm_segments: SmallVec<[Entity; 2]>,
    pub joints: SmallVec<[RapierMultibodyJointHandle; 2]>,
}

impl StickmanArm {
    #[inline(always)]
    pub fn new(arm_segments: SmallVec<[Entity; 2]>) -> Self {
        Self {
            arm_segments,
            ..Default::default()
        }
    }

    //maybe:
    //  add a command for reaching for a specific point in space
    //  command for rotating arm
    //  a way to "grow" another arm segment
    //  
}
