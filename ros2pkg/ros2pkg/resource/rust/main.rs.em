use anyhow::Result;
use rclrs::{Context, CreateBasicExecutor, RclrsErrorFilter, SpinOptions};

/// Creates a ROS 2 context and node, prints a hello message,
/// then spins until shutdown.
fn main() -> Result<()> {
    let context: Context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let _node = executor.create_node("@(node_name)")?;
    println!("Hello from @(package_name) package!");
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
