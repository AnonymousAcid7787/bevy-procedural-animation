use std::f32::consts::PI;

use bevy::prelude::*;
use bevy_flycam::FlyCam;
use bevy_rapier3d::{prelude::*, rapier::prelude::{JointLimits, MotorModel}};
use crate::stickman::{SegmentInfo, StickmanCommandsExt};


macro_rules! drive_motor {
    (
        $joint:expr,
        $dir:expr,
        $joint_axis:expr,
        $sleeping:expr
    ) => {
        let default_limits = JointLimits::default();
        if let Some(motor) = $joint.motor($joint_axis) {
            let target_vel = motor.target_vel;
            let current_target_pos = motor.target_pos;
            let stiffness = motor.stiffness;
            let damping = motor.damping;
            let limits = $joint.limits($joint_axis).unwrap_or(&default_limits);
            
            let new_target_pos = f32::clamp(
                current_target_pos + $dir,
                limits.min * (1. + damping),
                limits.max
            );
            

            if $dir != 0. {
                $sleeping.sleeping = false;
            }
            
            $joint.set_motor(
                $joint_axis,
                new_target_pos,
                target_vel,
                stiffness,
                damping
            );
        }
    };
}


pub fn test_update(
    mut shoulder_joints: Query<(&mut MultibodyJoint, &mut Sleeping), With<UpperArm>>,
    mut elbow_joints: Query<(&mut MultibodyJoint, &mut Sleeping), (With<LowerArm>, Without<UpperArm>)>,
    keys: Res<Input<KeyCode>>,
) {
    let default_limits = JointLimits::default();
    let dir = 
        if keys.pressed(KeyCode::Up) { f32::to_radians(5.) }
        else if keys.pressed(KeyCode::Down) { f32::to_radians(-5.) }
        else { f32::to_radians(0.) };
    
    //shoulder control
    let x_control = keys.pressed(KeyCode::X);
    let y_control = keys.pressed(KeyCode::Y);
    let z_control = keys.pressed(KeyCode::Z);
    for(mut shoulder_joint, mut sleeping) in shoulder_joints.iter_mut() {
        let joint = &mut shoulder_joint.data;
        if joint.as_spherical().is_none() {continue;}

        if x_control {
            drive_motor!(joint, dir, JointAxis::AngX, sleeping);
        }
        if y_control {
            drive_motor!(joint, dir, JointAxis::AngY, sleeping);
        }
        if z_control {
            drive_motor!(joint, dir, JointAxis::AngZ, sleeping);
        }
    }
    
    //elbow control
    if x_control || y_control || z_control {return;}
    for (mut elbow_joint, mut sleeping) in elbow_joints.iter_mut() {
        let joint = &mut  elbow_joint.data;
        
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

pub fn spawn_cubes(
    keys: Res<Input<KeyCode>>,
    cam_transform: Query<&Transform, With<FlyCam>>,
    mut commands: Commands,
) {
    // spawning cubes
    if keys.just_pressed(KeyCode::J) {
        if let Ok(transform) = cam_transform.get_single() {
            commands.spawn((
                Collider::cuboid(0.1, 0.1, 0.1),
                RigidBody::Dynamic,
                GlobalTransform::from(transform.clone()),
                ColliderMassProperties::Mass(1.)
            ));
        }
    }
}


pub fn stickman_setup(
    mut commands: Commands
) {
    let scale = 1.;

    let radius = 0.03_f32 * scale;
    let torso_depth = 0.6_f32 * scale;
    let torso_len = torso_depth + radius*2.;
    let arm_depth = torso_depth;
    let upper_arm_len = (arm_depth*0.7) + radius*2.;
    let lower_arm_len = (arm_depth*0.49) + radius*2.;
    let thickness = radius;

    //motor params
    let target_pos = f32::to_radians(0.);
    let target_vel = f32::to_radians(10.);
    let stiffness = 1.;
    
    let damping = 0.03;
    //torso
    let fixed_movement = commands.spawn(RigidBody::Fixed).id();
    let torso = commands.spawn((
        RigidBody::Dynamic,
        MultibodyJoint::new(fixed_movement, FixedJoint::new())
    )).id();
    commands.add_segment_physics(
        torso,
        SegmentInfo {
            length: torso_len,
            thickness
        }
    );

    //upper arm
    let upper_arm = commands.spawn((
        RigidBody::Dynamic,
        UpperArm,
    )).id();
    commands.add_segment_physics(
        upper_arm,
        SegmentInfo {
            length: upper_arm_len,
            thickness
        }
    );

    //lower arm
    let lower_arm = commands.spawn((
        RigidBody::Dynamic,
        LowerArm,
    )).id();
    commands.add_segment_physics(
        lower_arm,
        SegmentInfo {
            length: lower_arm_len,
            thickness
        }
    );


    //joints
    let x_limits = [0., 2.*PI];
    let y_limits = [0_f32.to_radians(), 120_f32.to_radians()];
    let z_limits = [190_f32.to_radians(), 350_f32.to_radians()];
    let mut shoulder = SphericalJointBuilder::new()
        .motor(JointAxis::AngX, x_limits[0], 30_f32.to_radians(), stiffness, damping)
        .motor(JointAxis::AngY, y_limits[0], 30_f32.to_radians(), stiffness, damping)
        .motor(JointAxis::AngZ, z_limits[0], 30_f32.to_radians(), stiffness, damping)
        .motor_model(JointAxis::AngX, MotorModel::AccelerationBased)
        .motor_model(JointAxis::AngY, MotorModel::AccelerationBased)
        .motor_model(JointAxis::AngZ, MotorModel::AccelerationBased)
        .limits(JointAxis::AngX, x_limits)
        .limits(JointAxis::AngY, y_limits)
        .limits(JointAxis::AngZ, z_limits)
        .local_anchor1((torso_len/2.) * Vec3::Y)
        .local_anchor2((upper_arm_len/2.) * Vec3::Y)
        .build();
        shoulder.set_contacts_enabled(false);
        let shoulder: GenericJoint = shoulder.into();
    let mut elbow = RevoluteJointBuilder::new(Vec3::Z)
        .limits([10_f32.to_radians(), 150_f32.to_radians()])
        .motor(target_pos, target_vel, stiffness, damping)
        .motor_model(MotorModel::ForceBased)
        .build();
        elbow.set_contacts_enabled(false);

    
    commands.create_arm(
        upper_arm,
        lower_arm,
        Some(torso),
        elbow,
        Some(shoulder),
        true
    );
    
}

#[derive(Component)]
pub struct UpperArm;
#[derive(Component)]
pub struct LowerArm;