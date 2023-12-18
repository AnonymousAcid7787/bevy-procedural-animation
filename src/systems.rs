use bevy::prelude::*;
use bevy_rapier3d::{prelude::*, rapier::prelude::{JointLimits, MotorModel}};
use crate::stickman::{SegmentInfo, StickmanCommandsExt};

pub fn stickman_body_setup(
    mut commands: Commands,
) {

    let scale = 1.;

    let radius = 0.03_f32 * scale;
    let torso_depth = 0.6_f32 * scale;
    let torso_len = torso_depth + radius*2.;
    let arm_depth = torso_depth;
    let arm_len = arm_depth + radius*2.;
    let leg_depth = 0.75 * scale;
    let leg_len = leg_depth + radius*2.;
    let latitudes = 8;
    let longitudes = 16;
    let arm_segment_depth = arm_depth/2.;
    let arm_segment_len = arm_segment_depth+radius;

    //motor params
    let target_pos = f32::to_radians(0.);
    let target_vel = f32::to_radians(30.);
    let stiffness = 1.;
    let damping = 0.03;

    let upper_arm = commands.spawn((
        SegmentInfo {
            length: arm_len,
            thickness: radius
        },
        RigidBody::Dynamic,
    ))
    .id();

    let lower_arm = commands.spawn((
        SegmentInfo {
            length: arm_len,
            thickness: radius
        },
        RigidBody::Dynamic,
    ))
    .id();
    
    let mut joint = RevoluteJointBuilder::new(Vec3::Z)
        .limits([0., 150_f32.to_radians()])
        .motor(target_pos, target_vel, stiffness, damping)
        .motor_model(MotorModel::ForceBased)
        .build();
    joint.set_contacts_enabled(false);
    
    commands.spawn_arm(upper_arm, lower_arm, joint);
}

pub fn test_update(
    mut multibody_joints: Query<(&mut MultibodyJoint, &mut Sleeping)>,
    keys: Res<Input<KeyCode>>,
) {
    let dir = 
        if keys.pressed(KeyCode::Up) { f32::to_radians(5.) }
        else if keys.pressed(KeyCode::Down) { f32::to_radians(-5.) }
        else { f32::to_radians(0.) };
        

    let default_limits = JointLimits::default();

    for (mut multibody_joint, mut sleeping) in multibody_joints.iter_mut() {
        let joint = &mut  multibody_joint.data;
        
        //revolute joint
        if joint.as_revolute().is_some() {
            let joint = unsafe { joint.as_revolute_mut().unwrap_unchecked() };
            let motor = joint.motor().unwrap();
            let target_vel = motor.target_vel;
            let current_target_pos = motor.target_pos;
            let stiffness = motor.stiffness;
            let damping = motor.damping;
            let limits = joint.limits().unwrap_or(&default_limits);
            
            let new_target_pos = f32::clamp(
                current_target_pos + dir,
                limits.min * (1. + damping),
                limits.max
            );

            if dir != 0. {
                sleeping.sleeping = false;
            }
            
            joint.set_motor(
                new_target_pos,
                target_vel,
                stiffness,
                damping
            );
        }

    }

}
