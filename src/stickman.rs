use bevy::{prelude::*, ecs::system::Command, utils::HashMap};
use bevy_rapier3d::{
    prelude::RapierMultibodyJointHandle,
    dynamics::{RevoluteJointBuilder, RigidBody, Sleeping, MultibodyJoint, GenericJoint},
    rapier::dynamics::{JointAxis, MotorModel, MultibodyJointHandle},
    geometry::Collider, plugin::{RapierContext, systems::init_joints}
};
use smallvec::SmallVec;

use crate::utils::immutable_ref_to_mutable;

pub trait StickmanCommandsExt {
    
}

//might add more onto this component
#[derive(Component)]
pub struct ArmSegment {
    /// The arm segment above this one in the arm heirarchy
    upper_arm: Option<Entity>,
    /// The arm segment below this one in the arm heirarchy
    lower_arm: Option<Entity>,
}

impl ArmSegment {
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

impl StickmanArm {
    #[inline(always)]
    pub fn new(arm_segments: SmallVec<[Entity; 2]>) -> Self {
        Self {
            arm_segments,
            ..Default::default()
        }
    }

    /// Make an [`Arm`] that has newly-created arm segment entities.
    pub fn make_arm_unjointed(
        segments: Vec<SegmentInfo>,
        commands: &mut Commands
    ) -> Self {
        let mut arm_segments: SmallVec<[Entity; 2]> = SmallVec::with_capacity(segments.len());

        // creating segment entities with their physics components attached.
        for i in 0..segments.len() {
            let segment = &segments[i];
            let half_depth = (segment.length - segment.thickness*2.)/2.;

            arm_segments[i] = commands.spawn((
                RigidBody::Dynamic,
                Collider::capsule(Vec3::Y * -half_depth, Vec3::Y * half_depth, segment.thickness),
                Sleeping::default(),
            )).id();
        }

        // attaching ArmSegment components to the created entities
        for i in 0..arm_segments.len() {
            let lower_arm = if i+1 < arm_segments.len() { Some(arm_segments[i+1]) } else { None };
            let upper_arm = if i-1 >= 0 { Some(arm_segments[i-1]) } else { None };

            commands.get_entity(arm_segments[i]).unwrap().
                insert(ArmSegment {
                    upper_arm,
                    lower_arm,
                });
        }

        return Self {
            arm_segments,
            joints: SmallVec::default(),
        }
    }

    /// Connect all segments in this arm using the same `joint`.
    /// 
    /// This function will overwrite joint local anchors.
    pub fn joint_segments(
        &self,
        commands: &mut Commands,
        joint: impl Into<GenericJoint>,
    ) {
        let joint_into: GenericJoint = joint.into();
        for i in 1..self.arm_segments.len() {
            let upper_segment = self.arm_segments[i-1];
            let mut joint = joint_into.clone();

            // joint.set_local_anchor1(Vec3::Y * )
            commands.get_entity(self.arm_segments[i]).unwrap()
                .insert(MultibodyJoint::new(self.arm_segments[i-1], joint_into));
        }
    }

    pub fn make_arm(
        upper_arm: SegmentInfo, 
        lower_arm: SegmentInfo, 
        motor_params: MotorParams, 
        commands: &mut Commands
    ) -> Self {
        let MotorParams {
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
                ArmSegment::with_upper_arm(upper),
            )).id();

        commands.get_entity(upper).unwrap()
            .insert(ArmSegment::with_lower_arm(lower));


        Self::new(SmallVec::from_buf([upper, lower]))
    }

    //maybe:
    //  add a command for reaching for a specific point in space
    //  command for rotating arm
    //  a way to "grow" another arm segment
    //  

}

pub struct MotorParams {
    pub joint_axis: Option<JointAxis>,
    pub target_pos: f32,
    pub target_vel: f32,
    pub stiffness: f32,
    pub damping: f32
}

pub struct Shoulder {
    pub shoulder_segments: SmallVec<[Entity; 2]>,
    pub joints: SmallVec<[RapierMultibodyJointHandle; 2]>,
}

impl Shoulder {
    pub fn make_shoulder(
        torso: SegmentInfo,
        arm: StickmanArm,
        commands: &mut Commands,
    ) {

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
fn test_thing() {
    
}


pub struct SpawnArm {
    upper_arm: Entity,
    lower_arm: Entity,
    joint: GenericJoint,
}

impl Command for SpawnArm {
    fn apply(self, world: &mut World) {
        {//upper arm
            let mut upper_arm = world.get_entity_mut(self.upper_arm).unwrap();

            let upper_arm_info = upper_arm
                .get::<SegmentInfo>()
                .unwrap()
                .clone();

            {//creating colliders if the segment doesn't have colliders
                if !upper_arm.contains::<Collider>() {
                    let half_upper_depth = (upper_arm_info.length - upper_arm_info.thickness*2.)/2.;
                    
                    upper_arm.insert((
                        Collider::capsule(Vec3::Y * -half_upper_depth, Vec3::Y * half_upper_depth, upper_arm_info.thickness),
                        Sleeping::default(),
                    ));
                }
                let half_lower_depth = (lower_arm_info.length - lower_arm_info.thickness*2.)/2.;
            }

        }

        {//lower arm
            let mut lower_arm = world.get_entity_mut(self.lower_arm).unwrap();

            let lower_arm_info = lower_arm
                .set_parent(self.upper_arm)
                .get::<SegmentInfo>()
                .unwrap()
                .clone();

            {//setting rapier joint handle
                let mut context = world.resource_mut::<RapierContext>();
                let context = &mut *context;
                let entity2body = context.entity2body();
                let scale = context.physics_scale();

                let target = entity2body.get(&self.lower_arm);
                let source = entity2body.get(&self.upper_arm);

                if let (Some(target), Some(source)) = (target, source) {
                    if let Some(handle) = context.multibody_joints.insert(
                        *source,
                        *target,
                        self.joint.into_rapier(scale),
                        true,
                    ) {
                        //SAFETY: This system has exclusive access of the enitre rapier context, so this should be fine.
                        unsafe {
                            immutable_ref_to_mutable(context.entity2multibody_joint()).insert(self.lower_arm, handle);
                        }
                        world
                            .entity_mut(self.lower_arm)
                            .insert(RapierMultibodyJointHandle(handle));
                    } else {
                        error!("Failed to create multibody joint: loop detected.")
                    }
                }
            }
            
        }
    }
}

#[derive(Component)]
pub struct Arm {
    pub upper_arm: Entity,
    pub lower_arm: Entity,
    pub joint: RapierMultibodyJointHandle,
}