use crossbeam::channel::unbounded;
use r2r::geometry_msgs::msg::{Point, Pose, Quaternion, Transform, TransformStamped, Vector3};
use r2r::std_msgs::msg::Header;
use r2r::tf2_msgs::msg::TFMessage;
use r2r::visualization_msgs::msg::{
    InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker,
};
use r2r::QosProfile;
use r2r_interactive_markers::InteractiveMarkerServer;
use r2r_regular_markers::RegularMarkerServer;
use std::sync::{Arc, Mutex};

/// Node identifier
pub static NODE_ID: &'static str = "teaching_markers_server";

/// Default feedback callback value
const DEFAULT_FEEDBACK_CB: u8 = 255;

#[derive(Clone)]
/// A struct representing a teaching marker in the interactive marker server.
pub struct TeachingMarkerServer {
    // markers: Vec<Markers>,
    interactive_marker_server: InteractiveMarkerServer,
    regular_marker_server: RegularMarkerServer
        // More fields can be added here if needed
}

#[derive(PartialEq)]
/// Enum representing the axes X, Y, and Z.
enum Axis {
    X,
    Y,
    Z,
}

/// Normalizes the quaternion in place.
///
/// # Arguments
///
/// * `quaternion` - A mutable reference to the quaternion to normalize.
fn normalize_quaternion(quaternion: &mut Quaternion) {
    let norm = quaternion.x * quaternion.x
        + quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
        + quaternion.w * quaternion.w;
    let s = norm.powf(-0.5);
    quaternion.x *= s;
    quaternion.y *= s;
    quaternion.z *= s;
    quaternion.w *= s;
}

/// Prepares an interactive marker control with the specified parameters.
///
/// # Arguments
///
/// * `name` - The name of the control.
/// * `interaction_mode` - The interaction mode for the control.
/// * `axis` - The axis along which the control operates.
///
/// # Returns
///
/// An `InteractiveMarkerControl` configured with the given parameters.
fn prepare_control(
    name: &str,
    interaction_mode: u8,
    axis: Axis,
    // marker: Option<Marker>,
) -> InteractiveMarkerControl {
    let mut control = InteractiveMarkerControl::default();
    control.orientation = Quaternion {
        w: 1.0,
        x: if axis == Axis::X { 1.0 } else { 0.0 },
        y: if axis == Axis::Y { 1.0 } else { 0.0 },
        z: if axis == Axis::Z { 1.0 } else { 0.0 },
    };
    control.always_visible = true;
    normalize_quaternion(&mut control.orientation);
    control.name = name.to_string();
    control.interaction_mode = interaction_mode;
    // if let Some(marker) = marker {
    //     control.markers.push(marker);
    // }
    control
}

impl TeachingMarkerServer {
    /// Creates a new `TeachingMarkerServer`.
    ///
    /// # Arguments
    ///
    /// * `name` - A topic namespace for the teaching marker server.
    /// * `node` - A shared reference to the ROS node.
    ///
    /// # Remarks
    ///
    /// This function initializes the interactive marker server and sets up publishers.
    pub fn new(name: &str, node: Arc<Mutex<r2r::Node>>) -> Self {

        let arc_node_clone = node.clone();
        let interactive_marker_server = InteractiveMarkerServer::new(name, arc_node_clone);
        let arc_node_clone = node.clone();
        let regular_marker_server = RegularMarkerServer::new("teaching_marker_server", name, arc_node_clone);

        TeachingMarkerServer {
            interactive_marker_server,
            regular_marker_server
        }
    }

    pub fn insert(&self, name: String, spawn_at: String, spawn_at_pose: Option<Pose>, regular_marker: Option<Marker>, node: Arc<Mutex<r2r::Node>>) {
        // Create the interactive marker
        let marker = Self::create_marker(&name, &spawn_at, spawn_at_pose.clone());

        // Set up a publisher for the TF messages with transient local QoS
        let arc_node_clone = node.clone();
        let publisher = arc_node_clone
            .lock()
            .unwrap()
            .create_publisher::<TFMessage>(
                "tf_static",
                QosProfile::transient_local(QosProfile::default()),
            )
            .unwrap();

        // Publish the initial transform before waiting for the feedback from RViz
        // let mut init_transform = TransformStamped::default();
        let mut init_transform = match spawn_at_pose {
            Some(p) => {
                let mut t = TransformStamped::default();
                t.transform = Transform {
                    translation: Vector3 { 
                        x: p.position.x, 
                        y: p.position.y, 
                        z: p.position.z 
                    },
                    rotation: Quaternion { 
                        x: p.orientation.x, 
                        y: p.orientation.y, 
                        z: p.orientation.z, 
                        w: p.orientation.w 
                    },
                };
                t
            },
            None => TransformStamped::default()
        };
        init_transform.child_frame_id = name.to_string();
        init_transform.header.frame_id = spawn_at.to_string();
        publisher.publish(
            &TFMessage { transforms: vec!(
                init_transform
            ) }
        ).unwrap();

        // Create an unbounded channel for communication
        let (tx, rx) = unbounded();

        // Start a thread to handle publishing the TF messages
        std::thread::spawn(move || {
            for data in rx.iter() {
                publisher.publish(&data).unwrap();
            }
        });

        // Insert the marker into the server
        self.interactive_marker_server.insert(marker);

        // Clone variables for the feedback callback
        let name_clone = name.clone();
        let tx_clone = tx.clone();

        // Define the feedback callback
        let feedback_cb = Arc::new(move |feedback: InteractiveMarkerFeedback| {
            let data = Self::process_feedback(&name_clone, &spawn_at, feedback);
            tx_clone.send(data).unwrap();
        });

        // Set the feedback callback for the marker
        self.interactive_marker_server.set_callback(&name, Some(feedback_cb.clone()), DEFAULT_FEEDBACK_CB);

        // Apply changes to publish updates
        self.interactive_marker_server.apply_changes();

        // If a marker is provided visualize it
        if let Some(marker) = regular_marker {
            self.regular_marker_server.insert(&name, marker);
            self.regular_marker_server.apply_changes();
        }

    }

    /// Creates an `InteractiveMarker` with controls for rotation and translation along all axes.
    ///
    /// # Arguments
    ///
    /// * `name` - The name of the marker.
    /// * `spawn_at` - The frame ID where the marker is to be spawned.
    /// * `spawn_at_pose` - The pose where we want to spawn the item at.
    ///
    /// # Returns
    ///
    /// An `InteractiveMarker` configured with controls.
    fn create_marker(name: &str, spawn_at: &str, spawn_at_pose: Option<Pose>) -> InteractiveMarker {
        let mut int_marker = InteractiveMarker::default();
        int_marker.header.frame_id = spawn_at.to_string();
        int_marker.name = format!("{name}");
        int_marker.description = format!("{name}");
        int_marker.scale = 0.3;
        int_marker.pose = match spawn_at_pose {
            Some(pose ) => pose,
            None => Pose {
                position: Point {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                orientation: Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
            }
        }; 
        
        // Add controls for rotation and movement along each axis
        for (name, interaction_mode, axis) in [
            (
                "rotate_x",
                InteractiveMarkerControl::ROTATE_AXIS as u8,
                Axis::X,
            ),
            ("move_x", InteractiveMarkerControl::MOVE_AXIS as u8, Axis::X),
            (
                "rotate_y",
                InteractiveMarkerControl::ROTATE_AXIS as u8,
                Axis::Y,
            ),
            ("move_y", InteractiveMarkerControl::MOVE_AXIS as u8, Axis::Y),
            (
                "rotate_z",
                InteractiveMarkerControl::ROTATE_AXIS as u8,
                Axis::Z,
            ),
            ("move_z", InteractiveMarkerControl::MOVE_AXIS as u8, Axis::Z),
        ] {
            int_marker.controls.push(prepare_control(
                name,
                interaction_mode,
                axis,
            ))
        }

        int_marker
    }

    /// Processes feedback from the interactive marker and generates a TF message.
    ///
    /// # Arguments
    ///
    /// * `name` - The name of the marker.
    /// * `spawn_at` - The frame ID where the marker is spawned.
    /// * `feedback` - The feedback received from the interactive marker.
    ///
    /// # Returns
    ///
    /// A `TFMessage` containing the updated transform based on the marker's feedback.
    ///
    /// # Remarks
    ///
    /// Currently, this function publishes directly to `/tf`. In future implementations,
    /// it can be modified to directly use a transform buffer like sms or r2r_transforms.
    fn process_feedback(
        name: &str,
        spawn_at: &str,
        feedback: InteractiveMarkerFeedback,
    ) -> TFMessage {
        // Get the current time
        let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
        let now = clock.get_now().unwrap();
        let time_stamp = r2r::Clock::to_builtin_time(&now);

        let mut transforms = vec![];

        // Create a TransformStamped message based on the feedback
        transforms.push(TransformStamped {
            header: Header {
                stamp: time_stamp.clone(),
                frame_id: spawn_at.to_string(),
            },
            child_frame_id: name.to_string(),
            transform: Transform {
                translation: Vector3 {
                    x: feedback.pose.position.x,
                    y: feedback.pose.position.y,
                    z: feedback.pose.position.z,
                },
                rotation: Quaternion {
                    x: feedback.pose.orientation.x,
                    y: feedback.pose.orientation.y,
                    z: feedback.pose.orientation.z,
                    w: feedback.pose.orientation.w,
                },
            },
        });

        TFMessage { transforms }
    }

}
