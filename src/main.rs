use std::time::Duration;

use bevy::{
    core_pipeline::{bloom::BloomSettings, clear_color::ClearColorConfig},
    input::{
        mouse::{MouseScrollUnit, MouseWheel},
        InputPlugin,
    },
    prelude::*,
    render::camera::{RenderTarget, ScalingMode},
    sprite::{MaterialMesh2dBundle, Mesh2dHandle},
    window::{PresentMode, WindowResized},
};
use bevy_rapier2d::prelude::*;
use bevy_tweening::{
    lens::{ColorMaterialColorLens, TransformScaleLens},
    Animator, AssetAnimator, EaseFunction, Lens, Tracks, Tween, TweeningPlugin,
};

const WALL_WIDTH: f32 = 100.0;

const BALL_MIN_VERTICES: usize = 32;

const BALL_MERGE_DURATION: f32 = 0.4;

const BALL_COLORS: [Color; 11] = [
    Color::hsl(0.0 * 36.0, 1.0, 0.5),
    Color::hsl(1.0 * 36.0, 1.0, 0.5),
    Color::hsl(2.0 * 36.0, 1.0, 0.5),
    Color::hsl(3.0 * 36.0, 1.0, 0.5),
    Color::hsl(4.0 * 36.0, 1.0, 0.5),
    Color::hsl(5.0 * 36.0, 1.0, 0.5),
    Color::hsl(6.0 * 36.0, 1.0, 0.5),
    Color::hsl(7.0 * 36.0, 1.0, 0.5),
    Color::hsl(8.0 * 36.0, 1.0, 0.5),
    Color::hsl(9.0 * 36.0, 1.0, 0.5),
    Color::hsl(0.0, 1.0, 1.0),
];

const BALL_RADIUS: f32 = 16.0;
// roughly sqrt(2)^i
const BALL_RADII: [f32; 11] = [
    BALL_RADIUS * 1.0,
    BALL_RADIUS * 1.41421356237,
    BALL_RADIUS * 2.0,
    BALL_RADIUS * 2.82842712475,
    BALL_RADIUS * 4.0,
    BALL_RADIUS * 5.65685424949,
    BALL_RADIUS * 8.0,
    BALL_RADIUS * 11.313708499,
    BALL_RADIUS * 16.0,
    BALL_RADIUS * 22.627416998,
    BALL_RADIUS * 32.0,
];

const PLAY_AREA_WIDTH: f32 = 600.0;
const PLAY_AREA_HEIGHT: f32 = 1e20;

const PIXELS_PER_LINE_SCROLLED: f32 = 20.0;

const PIXELS_PER_METER: f32 = 8.0;

#[derive(Component)]
struct Ball {
    level: usize,
}

#[derive(Component)]
/// semi-transparent ball shown when the cursor hovers over the screen
struct HoverBall;

// linearly interpolates distance by changing position
#[derive(Component)]
struct LerpDistance {
    target: Entity,
    timer: Timer,
    duration: Duration,
    start_distance: f32,
}

impl LerpDistance {
    /// Creates a LerpDistance component using the current start and end positions
    fn new(target: Entity, duration: Duration, start_distance: f32) -> Self {
        LerpDistance {
            target,
            timer: Timer::new(duration, TimerMode::Once),
            duration,
            start_distance,
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct TransformPositionZLens {
    /// Start value of the translation.
    pub start: f32,
    /// End value of the translation.
    pub end: f32,
}

impl Lens<Transform> for TransformPositionZLens {
    fn lerp(&mut self, target: &mut Transform, ratio: f32) {
        let value = self.start + (self.end - self.start) * ratio;
        target.translation.z = value;
    }
}

#[derive(Component)]
struct DestroyAfter(Timer);

#[derive(Component)]
/// Velocity determined by a change in position
struct ObservedVelocity {
    prev_position: Vec2,
    velocity: Vec2,
}

// TODO: make into bundle
#[derive(Component)]
struct Wall;

#[derive(Component)]
struct MainCamera;

#[derive(Component)]
struct BallCountText;

#[derive(Default, Resource)]
struct BallCount(usize);

#[derive(Default, Resource)]
struct Meshes {
    balls: Vec<Mesh2dHandle>,
}

fn setup(mut commands: Commands, windows: Res<Windows>, asset_server: Res<AssetServer>) {
    let window = windows.get_primary().unwrap();

    let mut camera_bundle = Camera2dBundle {
        projection: OrthographicProjection {
            scaling_mode: ScalingMode::FixedHorizontal(2.0 * PLAY_AREA_WIDTH),
            ..default()
        },
        camera_2d: Camera2d {
            clear_color: ClearColorConfig::Custom(Color::BLACK),
        },
        camera: Camera {
            hdr: true,
            ..default()
        },
        ..default()
    };
    camera_bundle.transform.translation.y = window.height() / window.width() * 500.0;
    commands
        .spawn(camera_bundle)
        .insert(BloomSettings {
            threshold: 0.9,
            intensity: 0.3,
            scale: 1.5,
            ..default()
        })
        .insert(MainCamera);

    commands
        .spawn(
            TextBundle::from_section(
                "0",
                TextStyle {
                    font: asset_server.load("fonts/OpenSans-Regular.ttf"),
                    font_size: 20.0,
                    color: Color::rgba(1.0, 0.84, 0.0, 0.4),
                },
            )
            .with_style(Style {
                position: UiRect::bottom(Val::Px(5.0)),
                ..default()
            }),
        )
        .insert(BallCountText);

    // left
    commands
        .spawn(Wall)
        .insert(Collider::cuboid(WALL_WIDTH, PLAY_AREA_HEIGHT))
        .insert(TransformBundle::from(Transform::from_xyz(
            -(PLAY_AREA_WIDTH + WALL_WIDTH),
            0.0,
            0.0,
        )));
    // right
    commands
        .spawn(Wall)
        .insert(Collider::cuboid(WALL_WIDTH, PLAY_AREA_HEIGHT))
        .insert(TransformBundle::from(Transform::from_xyz(
            PLAY_AREA_WIDTH + WALL_WIDTH,
            0.0,
            0.0,
        )));
    // bottom
    commands
        .spawn(Wall)
        .insert(Collider::cuboid(PLAY_AREA_WIDTH, WALL_WIDTH))
        .insert(TransformBundle::from(Transform::from_xyz(
            0.0,
            -WALL_WIDTH,
            0.0,
        )));
}

fn setup_meshes(mut assets: ResMut<Assets<Mesh>>, mut handles: ResMut<Meshes>) {
    handles.balls = BALL_RADII
        .iter()
        .map(|r| {
            assets
                .add(
                    shape::Circle {
                        radius: 0.5,
                        vertices: BALL_MIN_VERTICES * r.powf(0.4) as usize,
                    }
                    .into(),
                )
                .into()
        })
        .collect();
}

fn setup_hoverball(
    mut commands: Commands,
    mut materials: ResMut<Assets<ColorMaterial>>,
    meshes: Res<Meshes>,
    asset_server: Res<AssetServer>,
) {
    commands
        .spawn(MaterialMesh2dBundle {
            mesh: meshes.balls[0].clone(),
            material: materials.add(ColorMaterial {
                texture: Some(asset_server.load("textures/cursor.png")),
                color: *BALL_COLORS[0].clone().set_a(0.2),
            }),
            transform: Transform::from_scale(Vec3::splat(BALL_RADII[0] * 2.0)),
            ..default()
        })
        .insert(HoverBall);
}

// https://bevy-cheatbook.github.io/cookbook/cursor2world.html
fn world_space_cursor_position(
    // need to get window dimensions
    windows: Res<Windows>,
    // query to get camera transform
    query_camera: Query<(&Camera, &GlobalTransform), With<MainCamera>>,
) -> Option<Vec2> {
    // get the camera info and transform
    // assuming there is exactly one main camera entity, so query::single() is OK
    let (camera, camera_transform) = query_camera.single();

    // get the window that the camera is displaying to (or the primary window)
    let window = if let RenderTarget::Window(id) = camera.target {
        windows.get(id).unwrap()
    } else {
        windows.get_primary().unwrap()
    };

    // check if the cursor is inside the window and get its position or return None
    let Some(screen_pos) = window.cursor_position() else {
        return None;
    };

    // get the size of the window
    let window_size = Vec2::new(window.width() as f32, window.height() as f32);

    // convert screen position [0..resolution] to ndc [-1..1] (gpu coordinates)
    let ndc = (screen_pos / window_size) * 2.0 - Vec2::ONE;

    // matrix for undoing the projection and camera transform
    let ndc_to_world = camera_transform.compute_matrix() * camera.projection_matrix().inverse();

    // use it to convert ndc to world-space coordinates
    let world_pos = ndc_to_world.project_point3(ndc.extend(-1.0));

    // reduce it to a 2D value
    return Some(world_pos.truncate());
}

fn input_system(
    mut commands: Commands,
    mut ball_count: ResMut<BallCount>,
    mut materials: ResMut<Assets<ColorMaterial>>,
    meshes: Res<Meshes>,
    mouse_button_input: Res<Input<MouseButton>>,
    touch_input: Res<Touches>,
    windows: Res<Windows>,
    rapier_context: Res<RapierContext>,
    mut query_hoverball: Query<&mut Transform, With<HoverBall>>,
    query_camera: Query<(&Camera, &GlobalTransform), With<MainCamera>>,
) {
    let cursor_position = world_space_cursor_position(windows, query_camera);
    let touch_position = touch_input.iter_just_pressed().next().map(|t| t.position());

    let Some(position) = cursor_position.or(touch_position) else {
        return;
    };

    query_hoverball.single_mut().translation = position.extend(0.0);

    if rapier_context
        .intersection_with_shape(
            position,
            Rot::default(),
            &Collider::ball(BALL_RADII[0]),
            QueryFilter::default(),
        )
        .is_some()
    {
        return;
    }

    if mouse_button_input.pressed(MouseButton::Left) || touch_input.any_just_pressed() {
        commands
            .spawn(MaterialMesh2dBundle {
                mesh: meshes.balls[0].clone(),
                material: materials.add(ColorMaterial::from(BALL_COLORS[0])),
                transform: Transform::from_xyz(position.x, position.y, 1.)
                    .with_scale(Vec3::splat(BALL_RADII[0] * 2.0)),
                ..default()
            })
            .insert(Ball { level: 0 })
            .insert(ObservedVelocity {
                prev_position: position,
                velocity: Vec2::ZERO,
            })
            .insert(RigidBody::Dynamic)
            .insert(Collider::ball(0.5))
            .insert(ActiveEvents::COLLISION_EVENTS);
        //TODO: physics restitution and stuff

        ball_count.0 += 1;
    }
}

fn clamp_camera_y(mut transform: Mut<Transform>, window: &Window) {
    // limit camera y
    let min = window.height() / window.width() * PLAY_AREA_WIDTH;
    transform.translation.y = transform.translation.y.max(min);
}

fn window_resize_event(
    resize_event: Res<Events<WindowResized>>,
    mut query_camera: Query<&mut Transform, With<MainCamera>>,
    windows: Res<Windows>,
) {
    let reader = resize_event.get_reader();

    if !reader.is_empty(&resize_event) {
        clamp_camera_y(query_camera.single_mut(), windows.primary());
    }
}

// TODO: touch scrolling
fn scroll_event_system(
    mut mouse_scroll_event: EventReader<MouseWheel>,
    mut query_camera: Query<&mut Transform, With<MainCamera>>,
    windows: Res<Windows>,
    touch_input: Res<Touches>,
) {
    for mouse_wheel in mouse_scroll_event.iter() {
        let mut transform = query_camera.single_mut();

        match mouse_wheel.unit {
            MouseScrollUnit::Line => {
                transform.translation.y += mouse_wheel.y * PIXELS_PER_LINE_SCROLLED;
            }
            MouseScrollUnit::Pixel => {
                transform.translation.y += mouse_wheel.y;
            }
        }

        clamp_camera_y(transform, windows.primary());
        return;
    }

    for touch in touch_input.iter_just_pressed().chain(touch_input.iter_just_released()) {
        let mut transform = query_camera.single_mut();

        let difference = touch.position().y - touch.previous_position().y;

        transform.translation.y += difference;
        
        clamp_camera_y(transform, windows.primary());
        return;
    }
}

fn collision_event_system(
    mut commands: Commands,
    mut ball_count: ResMut<BallCount>,
    meshes: Res<Meshes>,
    mut query: Query<(
        &mut Ball,
        &mut Mesh2dHandle,
        &Transform,
        &Handle<ColorMaterial>,
        &ObservedVelocity,
    )>,
    mut collision_event: EventReader<CollisionEvent>,
) {
    for event in collision_event.iter() {
        match event {
            &CollisionEvent::Started(mut entity1, mut entity2, _) => {
                let Ok([a, b]) = query.get_many_mut([entity1, entity2]) else {
                    continue;
                };

                // compare ball levels
                if a.0.level != b.0.level && a.0.level < BALL_RADII.len() {
                    continue;
                }

                let [(pos1, vel1), (pos2, vel2)] = [
                    (a.2.translation.truncate(), a.4.velocity),
                    (b.2.translation.truncate(), b.4.velocity),
                ];

                // if entity1 is moving towards entity2 faster than entity2 is moving towards entity1
                // then consume entity1 and grow entity2, else vice versa
                let (mut ball2, mut mesh2, trans2, mat2, _) =
                    if vel1.dot(pos2 - pos1) > vel2.dot(pos1 - pos2) {
                        (entity1, entity2) = (entity2, entity1); // the ol switcheroo
                        a
                    } else {
                        b
                    };

                let mut e1_entity = commands.entity(entity1);
                e1_entity
                    .remove::<Collider>()
                    .remove::<RigidBody>()
                    .insert(LerpDistance::new(
                        entity2,
                        Duration::from_secs_f32(BALL_MERGE_DURATION),
                        BALL_RADII[ball2.level] * 2.0,
                    ))
                    .insert(DestroyAfter(Timer::from_seconds(
                        BALL_MERGE_DURATION,
                        TimerMode::Once,
                    )));
                ball_count.0 -= 1;

                ball2.level += 1;

                let ball_end_diameter = BALL_RADII[ball2.level] * 2.0;
                *mesh2 = meshes.balls[ball2.level].clone();

                let mut e2_entity = commands.entity(entity2);
                e2_entity.remove::<Animator<Transform>>();
                e2_entity.remove::<AssetAnimator<ColorMaterial>>();
                e2_entity.insert(Animator::new(Tracks::new([
                    Tween::new(
                        EaseFunction::QuadraticIn,
                        Duration::from_secs_f32(BALL_MERGE_DURATION),
                        TransformScaleLens {
                            start: trans2.scale,                 // INFO: do not set Z scale to 0.0
                            end: Vec3::splat(ball_end_diameter), // doubles surface area
                        },
                    ),
                    Tween::new(
                        EaseFunction::QuadraticIn,
                        Duration::from_secs_f32(BALL_MERGE_DURATION),
                        TransformPositionZLens {
                            start: (ball2.level as f32 - 1.0),
                            end: ball2.level as f32,
                        },
                    ),
                ])));
                e2_entity.insert(AssetAnimator::new(
                    mat2.clone(),
                    Tween::new(
                        EaseFunction::QuadraticIn,
                        Duration::from_secs_f32(BALL_MERGE_DURATION),
                        ColorMaterialColorLens {
                            start: BALL_COLORS[ball2.level - 1],
                            end: BALL_COLORS[ball2.level],
                        },
                    ),
                ));
                return;
            }
            _ => (),
        }
    }
}

fn text_update_system(
    ball_count: Res<BallCount>,
    mut query: Query<&mut Text, With<BallCountText>>,
) {
    let mut text = query.single_mut();
    text.sections[0].value = ball_count.0.to_string();
}

fn update_lerp_distance_system(
    time: Res<Time>,
    mut query_lerpers: Query<(Entity, &mut LerpDistance), With<Transform>>,
    mut query_transforms: Query<&mut Transform>,
) {
    query_lerpers.iter_mut().for_each(|(lerper, mut lerp)| {
        let alpha = lerp.timer.tick(time.delta()).remaining_secs() / lerp.duration.as_secs_f32()
            * lerp.start_distance;

        if let Ok([mut t1, t2]) = query_transforms.get_many_mut([lerper, lerp.target]) {
            let direction = (t2.translation - t1.translation).normalize();

            t1.translation = t2.translation - direction * alpha;
        };
    });
}

fn update_destroy_after_system(
    mut commands: Commands,
    time: Res<Time>,
    mut query: Query<(Entity, &mut DestroyAfter)>,
) {
    for (entity, mut destroy_after) in query.iter_mut() {
        if destroy_after.0.tick(time.delta()).finished() {
            commands.entity(entity).despawn_recursive();
        }
    }
}

fn update_observed_velocity_system(mut query: Query<(&mut ObservedVelocity, &Transform)>) {
    for (mut ov, t) in query.iter_mut() {
        ov.velocity = ov.prev_position - t.translation.truncate();
        ov.prev_position = t.translation.truncate();
    }
}

pub struct GamePlugin;

impl Plugin for GamePlugin {
    fn build(&self, app: &mut App) {
        fn panic_plugin_not_installed<P: Plugin>(app: &mut App) {
            if !app.is_plugin_added::<P>() {
                panic!(
                    "{} requires {} to work.",
                    stringify!(GamePlugin),
                    stringify!(P)
                );
            }
        }

        panic_plugin_not_installed::<TransformPlugin>(app);
        panic_plugin_not_installed::<InputPlugin>(app);
        panic_plugin_not_installed::<RapierPhysicsPlugin>(app);
        panic_plugin_not_installed::<TweeningPlugin>(app);

        app.init_resource::<BallCount>()
            .init_resource::<Meshes>()
            .add_startup_system(setup)
            .add_startup_system(setup_meshes)
            .add_startup_system(setup_hoverball.after(setup_meshes))
            .add_system(window_resize_event)
            .add_system(scroll_event_system)
            .add_system(update_observed_velocity_system)
            .add_system(collision_event_system.after(update_observed_velocity_system))
            .add_system(text_update_system)
            .add_system(update_lerp_distance_system)
            .add_system(update_destroy_after_system)
            .add_system(input_system);
    }
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            window: WindowDescriptor {
                title: "Ball Game".to_string(),
                present_mode: PresentMode::AutoVsync,
                ..default()
            },
            ..default()
        }))
        .add_plugin(RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(
            PIXELS_PER_METER,
        ))
        .add_plugin(TweeningPlugin)
        // .add_plugin(RapierDebugRenderPlugin::default()) // DEBUG
        .add_plugin(GamePlugin)
        .run();
}

// KNOWN BUGS:
// possible cause: closed window while running app
//
// 2023-01-02T13:36:11.495117Z ERROR wgpu_hal::vulkan::instance: VALIDATION [VUID-VkSwapchainCreateInfoKHR-imageExtent-01274 (0x7cd0911d)]
// Validation Error: [ VUID-VkSwapchainCreateInfoKHR-imageExtent-01274 ] Object 0: handle = 0x2506e745680, type = VK_OBJECT_TYPE_DEVICE; | MessageID = 0x7cd0911d | vkCreateSwapchainKHR() called with imageExtent = (1,1), which is outside the bounds returned by vkGetPhysicalDeviceSurfaceCapabilitiesKHR(): currentExtent = (0,0), minImageExtent = (0,0), maxImageExtent = (0,0). The Vulkan spec states: imageExtent must be between minImageExtent and maxImageExtent, inclusive, where minImageExtent and maxImageExtent are members of the VkSurfaceCapabilitiesKHR structure returned by vkGetPhysicalDeviceSurfaceCapabilitiesKHR for the surface (https://vulkan.lunarg.com/doc/view/1.3.216.0/windows/1.3-extensions/vkspec.html#VUID-VkSwapchainCreateInfoKHR-imageExtent-01274)
// 2023-01-02T13:36:11.495579Z ERROR wgpu_hal::vulkan::instance:   objects: (type: DEVICE, hndl: 0x2506e745680, name: ?)
