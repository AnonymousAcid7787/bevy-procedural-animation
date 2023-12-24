use bevy::prelude::*;
use bevy_flycam::FlyCam;
use bevy_rapier3d::{prelude::*, rapier::{prelude::{JointLimits, MotorModel}, dynamics::JointAxis}, parry::shape::Segment};
use crate::stickman::{SegmentInfo, StickmanCommandsExt};

pub fn stickman_body_setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut standard_materials: ResMut<Assets<StandardMaterial>>,
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
    let upper_arm_len = arm_len;
    let lower_arm_len = arm_len*0.7;
    let thickness = radius;

    //motor params
    let target_pos = f32::to_radians(0.);
    let target_vel = f32::to_radians(30.);
    let stiffness = 1.;
    let damping = 0.03;
    
    let torso;

    {//torso
        let fixed_movement = commands.spawn(RigidBody::Fixed).id();
        torso = commands.spawn((
            RigidBody::Dynamic,
            MultibodyJoint::new(fixed_movement, FixedJoint::new())
        ))
        .set_parent(fixed_movement)
        .id();

        commands.add_segment_physics(
            torso,
            SegmentInfo {
                length: torso_len,
                thickness: radius
            }
        );
    }

    //arm
    {
        //spawning arm segments
        let upper_arm = commands.spawn(RigidBody::Dynamic).id();
        commands.add_segment_physics(
            upper_arm,
            SegmentInfo {
                length: upper_arm_len,
                thickness
            }
        );
        let lower_arm = commands.spawn(RigidBody::Dynamic).id();
        commands.add_segment_physics(
            lower_arm,
            SegmentInfo {
                length: lower_arm_len,
                thickness
            }
        );

        //making joints
        let mut elbow = SphericalJointBuilder::new()
            .limits(JointAxis::AngX, [0., 0.])
            .limits(JointAxis::AngY, [0., 0.])
            .limits(JointAxis::AngZ, [0., 0.])
            // .limits([0., 150_f32.to_radians()])
            // .motor(target_pos, target_vel, stiffness, damping)
            // .motor_model(MotorModel::ForceBased)
            .build();
        let mut shoulder = SphericalJointBuilder::new()
        .limits(JointAxis::AngX, [0_f32.to_radians(), 180_f32.to_radians()])
        .limits(JointAxis::AngY, [60_f32.to_radians(), 300_f32.to_radians()])
        .limits(JointAxis::AngZ, [90_f32.to_radians(), 270_f32.to_radians()])
            .build();

        //disable joint contacts + joint anchors
        elbow
            .set_contacts_enabled(false)
            .set_local_anchor1((-upper_arm_len/2.) * Vec3::Y)
            .set_local_anchor2((lower_arm_len/2.) * Vec3::Y);
        shoulder
            .set_contacts_enabled(false)
            .set_local_anchor1((torso_len/2.) * Vec3::Y)
            .set_local_anchor2((upper_arm_len/2.) * Vec3::Y);

        //applying joints
        let upper_arm = commands.get_entity(upper_arm).unwrap()
            .set_parent(torso)
            .insert(MultibodyJoint::new(torso, SphericalJoint::new()))
            .id();

        let lower_arm = commands.get_entity(lower_arm).unwrap()
            .set_parent(upper_arm)
            .insert(MultibodyJoint::new(upper_arm, SphericalJoint::new()))
            .id();
    }

}

pub fn test_update(
    mut impulse_joints: Query<(&mut MultibodyJoint, &mut Sleeping)>,
    keys: Res<Input<KeyCode>>,
) {
    let dir = 
        if keys.pressed(KeyCode::Up) { f32::to_radians(5.) }
        else if keys.pressed(KeyCode::Down) { f32::to_radians(-5.) }
        else { f32::to_radians(0.) };

    let default_limits = JointLimits::default();

    for (mut impulse_joint, mut sleeping) in impulse_joints.iter_mut() {
        let joint = &mut  impulse_joint.data;
        
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

pub fn test_startup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
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

    let arm_shape = shape::Capsule {
        latitudes,
        longitudes,
        radius,
        depth: arm_depth,
        ..Default::default()
    };
    let torso_shape = shape::Capsule {
        latitudes,
        longitudes,
        radius,
        depth: torso_depth,
        ..Default::default()
    };
    let arm_mesh = meshes.add(arm_shape.into());
    let torso_mesh = meshes.add(torso_shape.into());
    
    let upper_arm_ent;
    //shoulder
    {
        //upper arm components + fixed
        let fixed_movement = commands.spawn(RigidBody::Fixed).id();
        let half_torso_depth = (torso_len - radius*2.)/2.;
        let mut torso = commands.spawn((
            RigidBody::Dynamic,
            Collider::capsule(Vec3::Y * -half_torso_depth, Vec3::Y * half_torso_depth, radius),
            Sleeping::default(),
        )); 
        torso.set_parent(fixed_movement)
            .insert(ImpulseJoint::new(fixed_movement, FixedJoint::default()));
        let torso = torso.id();

        //elbow joint
        let mut shoulder = SphericalJointBuilder::new()
            .build();
        shoulder.set_contacts_enabled(false);
        shoulder
            .set_local_anchor1((torso_len/2.) * Vec3::Y)
            .set_local_anchor2((arm_len/2.) * Vec3::Y);

        //upper arm + applying joint
        let half_upper_depth = (arm_len - radius*2.)/2.;
        let mut upper_arm = commands.spawn((
            RigidBody::Dynamic,
            Collider::capsule(Vec3::Y * -half_upper_depth, Vec3::Y * half_upper_depth, radius),
            Sleeping::default(),
            MultibodyJoint::new(torso, shoulder),
        ));
        upper_arm.set_parent(torso);

        upper_arm_ent = upper_arm.id();
    }

    //3rd segment
    {
        let mut elbow = RevoluteJoint::new(Vec3::Z);
        elbow
            .set_contacts_enabled(false)
            .set_local_anchor1((-arm_len/2.) * Vec3::Y)
            .set_local_anchor2((arm_len/2.) * Vec3::Y);

        let half_lower_depth = (arm_len - radius*2.)/2.;
        let lower_arm = commands.spawn((
            RigidBody::Dynamic,
            Collider::capsule(Vec3::Y * -half_lower_depth, Vec3::Y * half_lower_depth, radius),
            Sleeping::default(),
            ImpulseJoint::new(upper_arm_ent, elbow.clone())
        ))
        .set_parent(upper_arm_ent)
        .id();
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
    let arm_len = arm_depth + radius*2.;
    let upper_arm_len = (arm_depth*0.7) + radius*2.;
    let lower_arm_len = (arm_depth*0.49) + radius*2.;
    let thickness = radius;

    //motor params
    let target_pos = f32::to_radians(0.);
    let target_vel = f32::to_radians(30.);
    let stiffness = 1.;
    let damping = 0.03;

    //torso
    let fixed_movement = commands.spawn(RigidBody::Fixed).id();
    let torso = commands.spawn((
        RigidBody::Dynamic,
        ImpulseJoint::new(fixed_movement, FixedJoint::new())
    )).id();
    commands.add_segment_physics(
        torso,
        SegmentInfo {
            length: torso_len,
            thickness
        }
    );

    //upper arm
    let upper_arm = commands.spawn(RigidBody::Dynamic).id();
    commands.add_segment_physics(
        upper_arm,
        SegmentInfo {
            length: upper_arm_len,
            thickness
        }
    );

    //lower arm
    let lower_arm = commands.spawn(RigidBody::Dynamic).id();
    commands.add_segment_physics(
        lower_arm,
        SegmentInfo {
            length: lower_arm_len,
            thickness
        }
    );


    //joints
    let mut shoulder = SphericalJoint::new();
        shoulder.set_contacts_enabled(false);
        let shoulder: GenericJoint = shoulder.into();
    let mut elbow = RevoluteJointBuilder::new(Vec3::Z)
        .limits([0., 150_f32.to_radians()])
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
