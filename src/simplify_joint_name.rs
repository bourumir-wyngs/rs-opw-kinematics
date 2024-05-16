use regex::Regex;

/// "User friendly" pre-processing of the joint names. If this does not work as expected,
/// the user should use the advanced function taking the joint names explicitly.
///
/// Simplify joint name: remove macro construct in the prefix, 
/// underscores and non-word characters, set lowercase.
pub fn preprocess_joint_name(joint_name: &str) -> String {
    // Create a regex to find the ${prefix} pattern
    let re_prefix = Regex::new(r"\$\{[^}]+\}").unwrap();
    // Replace the pattern with an empty string, effectively removing it
    let processed_name = re_prefix.replace_all(joint_name, "");

    // Create a regex to remove all non-alphanumeric characters, including underscores
    let re_non_alphanumeric = Regex::new(r"[^\w]|_").unwrap();
    let clean_name = re_non_alphanumeric.replace_all(&processed_name, "");

    let processed_name = discard_non_digit_joint_chars(clean_name.to_string());
    remove_before_joint(processed_name.to_lowercase())
} 

fn discard_non_digit_joint_chars(input: String) -> String {
    // Define the regular expression
    let re = Regex::new(r".*joint(\D*)\d.*").unwrap();

    // Find the first match of the pattern
    if let Some(captures) = re.captures(&input) {
        // Get the part marked as \D* which is the first capture group
        if let Some(non_digit_part) = captures.get(1) {
            let non_digit_str = non_digit_part.as_str();
            return input.replace(non_digit_str, "");
        }
    }
    input
}

fn remove_before_joint(s: String) -> String {
    // Use a case-insensitive search for the word 'joint'
    if let Some(pos) = s.to_lowercase().find("joint") {
        // If 'joint' is found, return the substring starting from 'joint'
        s[pos..].to_string()
    } else {
        // If 'joint' is not found, return the original string
        s
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_name_simplification() {
        assert_eq!(preprocess_joint_name("joint1"), "joint1", " simplification incorrect");
        assert_eq!(preprocess_joint_name("JOINT2"), "joint2", "JOINT_2 simplification incorrect");        
        assert_eq!(preprocess_joint_name("leftJOINT_2!"), "joint2", "leftJOINT_2! simplification incorrect");        
    }
}

