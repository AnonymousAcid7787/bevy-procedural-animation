use bevy::{prelude::*, ecs::system::Command};
use bevy_rapier3d::{
    dynamics::{Sleeping, MultibodyJoint, GenericJoint, ImpulseJoint, JointAxesMask},
    geometry::Collider
};

pub trait StickmanCommandsExt {
    fn create_arm(
        &mut self,
        upper_arm: Entity,
        lower_arm: Entity,
        torso_ent: Option<Entity>,
        arm_joint: impl Into<GenericJoint>,
        shoulder_joint: Option<GenericJoint>,
        auto_joint_anchors: bool,
    ) -> Entity;
    
    fn add_segment_physics(
        &mut self,
        entity: Entity,
        segment_info: SegmentInfo,
    ) -> &mut Self;
}

impl StickmanCommandsExt for Commands<'_, '_> {

    /// Create an arm with the upper and lower arms inputted. Returns an entity representing the new arm.
    #[inline]
    fn create_arm(
            &mut self,
            upper_arm: Entity,
            lower_arm: Entity,
            torso_ent: Option<Entity>,
            arm_joint: impl Into<GenericJoint>,
            shoulder_joint: Option<GenericJoint>,
            auto_joint_anchors: bool,
    ) -> Entity {
        if shoulder_joint.is_some() && torso_ent.is_none() {
            panic!("An arm cannot have a shoulder without a torso to attach to!")
        }
        if torso_ent.is_some() && shoulder_joint.is_none() {
            panic!("An arm cannot attach to a torso without a specified shoulder joint!")
        }
        self.add(CreateArm {
            upper_arm,
            lower_arm,
            torso_ent,
            arm_joint: arm_joint.into(),
            shoulder_joint: 
                if let Some(shoulder_joint) = shoulder_joint {Some(shoulder_joint)}
                else {None},
            auto_joint_anchors,
        });

        return self.spawn(Arm {
            upper_arm,
            lower_arm,
            torso_ent: torso_ent
        }).id();
    }

    /// Creates a capsule collider for the given `entity` according to the information on the `segment_info`.
    /// 
    /// Adds the `segment_info` component to the `entity`.
    #[inline]
    fn add_segment_physics(
        &mut self,
        entity: Entity,
        segment_info: SegmentInfo,
    ) -> &mut Self {
        let half_segment_depth = (segment_info.length - segment_info.thickness*2.)/2.;
            
        self.get_entity(entity).unwrap().insert((
            Collider::capsule(Vec3::Y * -half_segment_depth, Vec3::Y * half_segment_depth, segment_info.thickness),
            Sleeping::default(),
            segment_info
        ));

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

#[derive(Component)]
pub struct Arm {
    pub upper_arm: Entity,
    pub lower_arm: Entity,
    /// The [`Torso`] entity that this arm is connected to. 
    /// Set this to [`None`] to have an arm without a connection to a torso.
    pub torso_ent: Option<Entity>,
}

#[derive(Component)]
pub struct Torso {
    /* to be implemented */
}

pub struct CreateArm {
    pub upper_arm: Entity,
    pub lower_arm: Entity,
    pub torso_ent: Option<Entity>,
    pub arm_joint: GenericJoint,
    pub shoulder_joint: Option<GenericJoint>,
    pub auto_joint_anchors: bool,
}

impl Command for CreateArm {
    fn apply(mut self, world: &mut World) {
        //setting joint anchors automatically
        if self.auto_joint_anchors {
            let mut torso_info = None;
            let upper_arm_info = Some(world.get::<SegmentInfo>(self.upper_arm).expect("auto joint anchors: upper_arm doesn't have SegmentInfo component!"));
            let lower_arm_info = Some(world.get::<SegmentInfo>(self.lower_arm).expect("auto joint anchors: lower_arm doesn't have SegmentInfo component!"));
            if let Some(torso_ent) = self.torso_ent {
                torso_info = Some(world.get::<SegmentInfo>(torso_ent).expect("auto joint anchors: torso doesn't have SegmentInfo component!"));
            }

            let upper_arm_info = upper_arm_info.unwrap();
            let lower_arm_info = lower_arm_info.unwrap();

            //setting joint anchors
            self.arm_joint
                .set_local_anchor1((-upper_arm_info.length/2.) * Vec3::Y)
                .set_local_anchor2((lower_arm_info.length/2.) * Vec3::Y);

            //shoulder anchors
            if self.shoulder_joint.is_some() {
                let torso_info = torso_info.unwrap();
                self.shoulder_joint.as_mut().unwrap()
                    .set_local_anchor1((torso_info.length/2.) * Vec3::Y)
                    .set_local_anchor2((upper_arm_info.length/2.) * Vec3::Y);
            }
        }
        
        //shoulder
        if let Some(torso_ent) = self.torso_ent {
            world.get_entity_mut(self.upper_arm).unwrap()
                .insert(MultibodyJoint::new(torso_ent, self.shoulder_joint.unwrap()))
                .set_parent(torso_ent);
        }

        //elbow
        let mut lower_arm_mut = world.get_entity_mut(self.lower_arm).unwrap();
        lower_arm_mut
            .insert(MultibodyJoint::new(self.upper_arm, self.arm_joint))
            .set_parent(self.upper_arm);
        if let Some(torso_ent) = self.torso_ent {
            lower_arm_mut.insert(ImpulseJoint::new(torso_ent, GenericJoint::new(JointAxesMask::empty())));
        }
    }
}
