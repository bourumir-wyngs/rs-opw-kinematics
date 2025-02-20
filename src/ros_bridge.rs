use std::thread::sleep;
use anyhow::{Error, Result};

use parry3d::shape::TriMesh;
use rs_cxx_ros2_opw_bridge::sender::Sender;
use crate::organized_point::OrganizedPoint;

pub struct RosSender {
    sender: Sender
}

impl Default for RosSender {
    fn default() -> Self {
        Self {
            sender: Sender::new("127.0.0.1", 5555),
        }
    }
}

impl RosSender {
    pub fn new(ip: &str, port: u16) -> Self {
        Self {
            sender: Sender::new(ip, port)
        }
    }

    pub fn cloud(
        &self,
        points: &Vec<OrganizedPoint>,
        color: (i32, i32, i32),
        transparency: f32,
    ) -> Result<()> {
        let points: Vec<parry3d::math::Point<f32>> = points
            .iter()
            .map(|p| (p.point)).collect();
        self.points(&points, color, transparency)?;
        sleep(std::time::Duration::from_millis(200));
        Ok(())
    }

    pub fn points(
        &self,
        points: &Vec<parry3d::math::Point<f32>>,
        color: (i32, i32, i32),
        transparency: f32,
    ) -> Result<()> {
        let points: Vec<(f32, f32, f32)> = points
            .iter()
            .map(|p| (p.x, p.y, p.z))
            .collect();
        self.tuples(&points, color, transparency)?;
        sleep(std::time::Duration::from_millis(200));
        Ok(())
    }
    

    fn tuples(&self, points: &Vec<(f32, f32, f32)>, color: (i32, i32, i32), transparency: f32) -> Result<()>{
        match self.sender.send_point_cloud_message(
            &points,
            &vec![],
            (color.0 as u8, color.1 as u8, color.2 as u8),
            transparency,
        ) {
            Ok(_) => {
                println!("Pose message sent successfully.");
                Ok(())
            }
            Err(err) => {
                eprintln!("Failed to send pose message: {}", err);
                Err(Error::from(err))
            }
        }
    }

    pub fn mesh(&self, mesh: &TriMesh, color: (i32, i32, i32), transparency: f32) -> Result<()> {
        let points: Vec<(f32, f32, f32)> = mesh.vertices().iter().map(|p| (p.x, p.y, p.z)).collect();
        let triangles = mesh
            .indices()
            .iter()
            .map(|t| (t[0] as u32, t[1] as u32, t[2] as u32))
            .collect();

        match self.sender.send_point_cloud_message(
            &points,
            &triangles,
            (color.0 as u8, color.1 as u8, color.2 as u8),
            transparency,
        ) {
            Ok(_) => {
                println!("Pose message sent successfully.");
            }
            Err(err) => {
                eprintln!("Failed to send pose message: {}", err);
            }
        }
        sleep(std::time::Duration::from_millis(200));
        Ok(())
    }
}