[package]
name = "@project_name"
version = "0.0.1"
authors = ["@maintainer_name <@maintainer_email>"]
edition = "2021"

[[bin]]
name = "@project_name"
path = "src/main.rs"

[dependencies]
anyhow = {version = "1", features = ["backtrace"]}
rclrs = "*"

# This specific version is compatible with Rust 1.75
backtrace = "=0.3.74"

@[if dependencies]@
@[  for dep in dependencies]@
@dep = "*"
@[  end for]@
@[else]@
# uncomment the following section in order to fill in
# further dependencies manually.
# <dependency> = "*" 
@[end if]@
