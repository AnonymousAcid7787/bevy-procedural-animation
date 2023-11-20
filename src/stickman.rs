use bevy::prelude::*;
use bevy_rapier3d::{prelude::RapierMultibodyJointHandle, dynamics::{RevoluteJointBuilder, RigidBody, Sleeping, MultibodyJoint}, rapier::dynamics::{JointAxis, MotorModel}, geometry::Collider};
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
    left_arm: Entity,
    right_arm: Entity,
}

//Stickman arm component (parent of both upper and lower arm(s))
#[derive(Component, Default, Clone)]
pub struct StickmanArm {
    // List of arm segments in the heirarchy (from upper to lower)
    pub arm_segments: SmallVec<[Entity; 2]>,
    pub joints: SmallVec<[RapierMultibodyJointHandle; 2]>,
}

pub struct ArmMotorParams {
    pub joint_axis: Option<JointAxis>,
    pub target_pos: f32,
    pub target_vel: f32,
    pub stiffness: f32,
    pub damping: f32
}

impl StickmanArm {
    #[inline(always)]
    pub fn new(arm_segments: SmallVec<[Entity; 2]>) -> Self {
        Self {
            arm_segments,
            ..Default::default()
        }
    }

    pub fn make_arm(
        upper_arm: SegmentInfo, 
        lower_arm: SegmentInfo, 
        motor_params: ArmMotorParams, 
        commands: &mut Commands
    ) -> Self {
        let ArmMotorParams {
            joint_axis: _,
            target_pos,
            target_vel,
            stiffness,
            damping
        } = motor_params;

        let mut joint = RevoluteJointBuilder::new(Vec3::Z)
            .local_anchor1(Vec3::Y * -upper_arm.length/2.)
            .local_anchor2(Vec3::Y * upper_arm.length/2.)
            .limits([0., 150_f32.to_radians()])
            .motor(target_pos, target_vel, stiffness, damping)
            .motor_model(MotorModel::ForceBased)
            .build();
        joint.set_contacts_enabled(false);

        let half_upper_depth = (upper_arm.length - upper_arm.thickness*2.)/2.;
        let half_lower_depth = (lower_arm.length - lower_arm.thickness*2.)/2.;
            
        let upper = commands.spawn((
            RigidBody::Dynamic,
            Collider::capsule(Vec3::Y * -half_upper_depth, Vec3::Y * half_upper_depth, upper_arm.thickness),
            Sleeping::default(),
        )).id();

        let lower = commands.spawn_empty()
            .set_parent(upper)
            .insert((
                RigidBody::Dynamic,
                Collider::capsule(Vec3::Y * -half_lower_depth, Vec3::Y * half_lower_depth, lower_arm.thickness),
                Sleeping::default(),
                MultibodyJoint::new(upper, joint),
                StickmanArmSegment::with_upper_arm(upper),
            )).id();

        commands.get_entity(upper).unwrap()
            .insert(StickmanArmSegment::with_lower_arm(lower));


        Self::new(SmallVec::from_buf([upper, lower]))
    }

    //maybe:
    //  add a command for reaching for a specific point in space
    //  command for rotating arm
    //  a way to "grow" another arm segment
    //  
}

pub struct SegmentInfo {
    pub thickness: f32,
    pub length: f32,
}
