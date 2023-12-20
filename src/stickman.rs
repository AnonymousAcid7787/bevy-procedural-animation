use bevy::{prelude::*, ecs::system::Command};
use bevy_rapier3d::{
    prelude::RapierMultibodyJointHandle,
    dynamics::{Sleeping, MultibodyJoint, GenericJoint},
    geometry::Collider
};

pub trait StickmanCommandsExt {
    fn create_arm(
        &mut self,
        upper_arm: Entity,
        lower_arm: Entity,
        joint: impl Into<GenericJoint>,
    ) -> &mut Self;
}

impl StickmanCommandsExt for Commands<'_, '_> {
    fn create_arm(
            &mut self,
            upper_arm: Entity,
            lower_arm: Entity,
            joint: impl Into<GenericJoint>,
        ) -> &mut Self {
        self.add(CreateArm {
            upper_arm,
            lower_arm,
            joint: joint.into(),
        });

        return self;
    }
}

#[derive(Component, Clone)]
pub struct SegmentInfo {
    pub thickness: f32,
    pub length: f32,
}


/// Physics stuff:
/// joints
/// colliders
/// 
/// Bevy stuff:
/// Entities
/// parenting
/// 
/// 
/// Arm:
/// Entity representing the arm as a whole
/// Multiple entities that represent the arm segments
///     Capsule collider? Any collider? 
///     Maybe make it possible to specify the kind of collider, but make the arm length adjustable for anchors.
/// 
/// Revolute joint (probably will change to a joint with 2 DOFs once they are supported)
///     Use joint limits to prevent the arm from clipping into the torso. 
///     Not sure if joint limits only limit motor angles or if they actually apply an angular constraint
/// 
/// 
/// Shoulder:
/// Needs a single torso 
/// Spherical joint
///     Use joint limits to prevent the arm from clipping into the torso. 
///     Not sure if joint limits only limit motor angles or if they actually apply an angular constraint


pub struct CreateArm {
    pub upper_arm: Entity,
    pub lower_arm: Entity,
    pub joint: GenericJoint,
}

impl Command for CreateArm {
    fn apply(mut self, world: &mut World) {
        let upper_len;
        let lower_len;

        {//upper arm
            let mut upper_arm = world.get_entity_mut(self.upper_arm).unwrap();

            let upper_arm_info = upper_arm
                .get::<SegmentInfo>()
                .unwrap();
            upper_len = upper_arm_info.length;

            //creating colliders if the segment doesn't have colliders
            if !upper_arm.contains::<Collider>() {
                let half_upper_depth = (upper_arm_info.length - upper_arm_info.thickness*2.)/2.;
                
                upper_arm.insert((
                    Collider::capsule(Vec3::Y * -half_upper_depth, Vec3::Y * half_upper_depth, upper_arm_info.thickness),
                    Sleeping::default(),
                ));
            }

        }

        {//lower arm
            let mut lower_arm = world.get_entity_mut(self.lower_arm).unwrap();

            let lower_arm_info = lower_arm
                .get::<SegmentInfo>()
                .unwrap();
            lower_len = lower_arm_info.length;

            //creating colliders if the segment doesn't have colliders
            if !lower_arm.contains::<Collider>() {
                let half_lower_depth = (lower_arm_info.length - lower_arm_info.thickness*2.)/2.;
                
                lower_arm.insert((
                    Collider::capsule(Vec3::Y * -half_lower_depth, Vec3::Y * half_lower_depth, lower_arm_info.thickness),
                    Sleeping::default(),
                ));
            }

            //setting joint anchors
            self.joint
                .set_local_anchor1((-upper_len/2.) * Vec3::Y)
                .set_local_anchor2((lower_len/2.) * Vec3::Y)
                .set_contacts_enabled(false);
            
            //setting rapier joint handle
            lower_arm
                .set_parent(self.upper_arm)
                .insert(MultibodyJoint::new(self.upper_arm, self.joint));
        }
        
    }
}

#[derive(Component)]
pub struct Arm {
    pub upper_arm: Entity,
    pub lower_arm: Entity,
    pub joint: RapierMultibodyJointHandle,
}

pub struct CreateShoulder {
    pub arm_ent: Entity,
    pub torso_ent: Entity,
    pub joint: RapierMultibodyJointHandle,
}

impl Command for CreateShoulder {
    fn apply(self, world: &mut World) {
        
    }
}