const VERSION: &str = "1.2.0";

#[cfg(feature = "allow_filesystem")]
fn main() {
    use clap::{Parser};
    use std::fs::File;
    use std::io::{self, Read};
    use rs_opw_kinematics::constraints::BY_PREV;
    use rs_opw_kinematics::kinematic_traits::JOINTS_AT_ZERO;
    use rs_opw_kinematics::urdf;

    fn print_usage() {
        println!("This command line utility extracts OPW parameters \
              \nfor OPW robots from URDF or XACRO files. \
              \n\nIf XACRO file is used, it must contain joint descriptions directly, file \
              \ninclusions are not followed. The function ${{radians(degrees)}} is supported. \
              \nWhile both parameters and constraints are printed out, constraints are not \
              \npart of the OPW parameters. \
              \n\nThis tool is Free software under BSD 3, hosted in repository \
              \nhttps://github.com/bourumir-wyngs/rs-opw-kinematics\n");
        println!("Usage: rs-opw-kinematics urdf_file.urdf");
    }    

    fn read_file(file_name: &str) -> io::Result<String> {
        let mut file = File::open(file_name)?;
        let mut content = String::new();
        file.read_to_string(&mut content)?;
        Ok(content)
    }

    #[derive(Parser)]
    #[command(author, version, about, long_about = None)]
    struct Cli {
        /// The file to read
        file: Option<String>,
    }

    println!("rs-opw-kinematics URDF extractor {VERSION} by Bourumir Wyngs.\n");
    
    let cli = Cli::parse();

    if let Some(file_name) = cli.file {
        match read_file(&file_name) {
            Ok(content) => {
                match urdf::from_urdf(content, &None) {
                    Ok(opw) => {
                        println!("OPW parameters:\n{}\nJoint limits:\n{}",
                                &opw.parameters(&JOINTS_AT_ZERO).to_yaml(),
                                 opw.constraints(BY_PREV).to_yaml());                        
                        
                    },
                    Err(e) => println!("Error: {}", e),
                }
            },
            Err(e) => println!("Error reading file: {}", e),
        }
    } else {
        print_usage();
    }
}

#[cfg(not(feature = "allow_filesystem"))]
fn main() {
    println!("rs-opw-kinematics {VERSION}, CLI not built.");
}

