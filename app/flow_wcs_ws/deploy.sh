#!/bin/bash

###############################################################################
# Flow WCS Deployment Script
# 部署 Flow WCS v2 系統
###############################################################################

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Paths
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$SCRIPT_DIR"
BUILD_DIR="$WORKSPACE_DIR/build"
INSTALL_DIR="$WORKSPACE_DIR/install"
FLOWS_DIR="/app/config/wcs/flows"
CHECKPOINT_DIR="/app/data/flow_wcs/checkpoints"

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  Flow WCS v2 Deployment Script${NC}"
echo -e "${BLUE}========================================${NC}"

# Function to print colored messages
print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to check if we're in container
check_environment() {
    if [ ! -f /.dockerenv ]; then
        print_error "This script must be run inside the AGVC container!"
        print_info "Please run: docker compose -f docker-compose.agvc.yml exec agvc_server bash"
        exit 1
    fi
    
    if [ "$CONTAINER_TYPE" != "agvc" ]; then
        print_error "This script must be run in the AGVC container!"
        exit 1
    fi
    
    print_info "Environment check passed - Running in AGVC container"
}

# Function to setup directories
setup_directories() {
    print_info "Setting up directories..."
    
    # Create flows directory
    if [ ! -d "$FLOWS_DIR" ]; then
        print_info "Creating flows directory: $FLOWS_DIR"
        mkdir -p "$FLOWS_DIR"
    fi
    
    # Create checkpoint directory
    if [ ! -d "$CHECKPOINT_DIR" ]; then
        print_info "Creating checkpoint directory: $CHECKPOINT_DIR"
        mkdir -p "$CHECKPOINT_DIR"
    fi
    
    # Set permissions
    chmod 755 "$FLOWS_DIR"
    chmod 755 "$CHECKPOINT_DIR"
    
    print_info "Directories setup complete"
}

# Function to load ROS 2 environment
load_ros_environment() {
    print_info "Loading ROS 2 environment..."
    
    # Source ROS 2 setup
    source /opt/ros/jazzy/setup.bash
    
    # Source local setup if exists
    if [ -f "$INSTALL_DIR/setup.bash" ]; then
        source "$INSTALL_DIR/setup.bash"
        print_info "Loaded local installation"
    fi
    
    # Set RMW implementation
    export RMW_IMPLEMENTATION=rmw_zenohd
    
    print_info "ROS 2 environment loaded"
}

# Function to build Flow WCS
build_flow_wcs() {
    print_info "Building Flow WCS workspace..."
    
    cd "$WORKSPACE_DIR"
    
    # Clean previous build if requested
    if [ "$1" == "--clean" ]; then
        print_info "Cleaning previous build..."
        rm -rf "$BUILD_DIR" "$INSTALL_DIR"
    fi
    
    # Build with colcon
    print_info "Running colcon build..."
    colcon build --packages-select flow_wcs \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --event-handlers console_direct+
    
    if [ $? -eq 0 ]; then
        print_info "Build successful!"
    else
        print_error "Build failed!"
        exit 1
    fi
}

# Function to run tests
run_tests() {
    print_info "Running Flow WCS tests..."
    
    cd "$WORKSPACE_DIR"
    
    # Run Python tests
    print_info "Running Python unit tests..."
    python3 -m pytest src/flow_wcs/test/ -v
    
    if [ $? -eq 0 ]; then
        print_info "All tests passed!"
    else
        print_warn "Some tests failed - please review"
    fi
}

# Function to deploy flow files
deploy_flow_files() {
    print_info "Deploying flow files..."
    
    # Check if there are example flows to deploy
    EXAMPLE_FLOWS_DIR="$WORKSPACE_DIR/src/flow_wcs/config/flows"
    
    if [ -d "$EXAMPLE_FLOWS_DIR" ]; then
        print_info "Copying example flows to $FLOWS_DIR"
        cp -n "$EXAMPLE_FLOWS_DIR"/*.yaml "$FLOWS_DIR/" 2>/dev/null || true
        print_info "Example flows deployed (existing files not overwritten)"
    fi
    
    # List deployed flows
    print_info "Deployed flows:"
    ls -la "$FLOWS_DIR"/*.yaml 2>/dev/null || print_warn "No flows found"
}

# Function to start Flow WCS
start_flow_wcs() {
    print_info "Starting Flow WCS system..."
    
    # Load environment
    load_ros_environment
    
    # Check if already running
    if pgrep -f "flow_wcs_node" > /dev/null; then
        print_warn "Flow WCS is already running"
        return
    fi
    
    # Start with launch file
    print_info "Launching Flow WCS..."
    ros2 launch flow_wcs flow_wcs_launch.py \
        flows_dir:="$FLOWS_DIR" \
        scan_interval:=10.0 \
        enable_monitor:=true &
    
    sleep 3
    
    # Verify startup
    if pgrep -f "flow_wcs_node" > /dev/null; then
        print_info "Flow WCS started successfully!"
        print_info "Monitoring logs at: ros2 topic echo /flow_wcs/status"
    else
        print_error "Failed to start Flow WCS"
        exit 1
    fi
}

# Function to stop Flow WCS
stop_flow_wcs() {
    print_info "Stopping Flow WCS system..."
    
    # Kill flow_wcs processes
    pkill -f "flow_wcs_node" 2>/dev/null || true
    pkill -f "flow_executor" 2>/dev/null || true
    pkill -f "flow_monitor" 2>/dev/null || true
    
    sleep 2
    
    # Verify stopped
    if pgrep -f "flow_wcs" > /dev/null; then
        print_warn "Some Flow WCS processes still running"
        print_info "Force stopping..."
        pkill -9 -f "flow_wcs" 2>/dev/null || true
    fi
    
    print_info "Flow WCS stopped"
}

# Function to check system status
check_status() {
    print_info "Checking Flow WCS status..."
    
    echo -e "\n${BLUE}=== Process Status ===${NC}"
    if pgrep -f "flow_wcs_node" > /dev/null; then
        echo -e "flow_wcs_node: ${GREEN}Running${NC}"
    else
        echo -e "flow_wcs_node: ${RED}Not running${NC}"
    fi
    
    if pgrep -f "flow_executor" > /dev/null; then
        echo -e "flow_executor: ${GREEN}Running${NC}"
    else
        echo -e "flow_executor: ${RED}Not running${NC}"
    fi
    
    if pgrep -f "flow_monitor" > /dev/null; then
        echo -e "flow_monitor: ${GREEN}Running${NC}"
    else
        echo -e "flow_monitor: ${RED}Not running${NC}"
    fi
    
    echo -e "\n${BLUE}=== ROS 2 Topics ===${NC}"
    ros2 topic list | grep flow_wcs || echo "No flow_wcs topics found"
    
    echo -e "\n${BLUE}=== Flow Files ===${NC}"
    ls -la "$FLOWS_DIR"/*.yaml 2>/dev/null | head -5 || echo "No flow files found"
    
    echo -e "\n${BLUE}=== Latest Logs ===${NC}"
    if [ -f /tmp/flow_wcs.log ]; then
        tail -5 /tmp/flow_wcs.log
    else
        echo "No log file found"
    fi
}

# Function to show usage
show_usage() {
    echo "Usage: $0 [COMMAND] [OPTIONS]"
    echo ""
    echo "Commands:"
    echo "  build       Build Flow WCS workspace"
    echo "  test        Run tests"
    echo "  deploy      Deploy flow files and setup"
    echo "  start       Start Flow WCS system"
    echo "  stop        Stop Flow WCS system"
    echo "  restart     Restart Flow WCS system"
    echo "  status      Check system status"
    echo "  full        Full deployment (build, test, deploy, start)"
    echo ""
    echo "Options:"
    echo "  --clean     Clean build (with 'build' or 'full' command)"
    echo "  --help      Show this help message"
}

# Main script logic
main() {
    # Check for help
    if [ "$1" == "--help" ] || [ "$1" == "-h" ]; then
        show_usage
        exit 0
    fi
    
    # Check environment
    check_environment
    
    # Parse command
    COMMAND="${1:-full}"
    shift || true
    
    case "$COMMAND" in
        build)
            load_ros_environment
            build_flow_wcs "$@"
            ;;
        test)
            load_ros_environment
            run_tests
            ;;
        deploy)
            setup_directories
            deploy_flow_files
            ;;
        start)
            start_flow_wcs
            ;;
        stop)
            stop_flow_wcs
            ;;
        restart)
            stop_flow_wcs
            start_flow_wcs
            ;;
        status)
            load_ros_environment
            check_status
            ;;
        full)
            load_ros_environment
            setup_directories
            build_flow_wcs "$@"
            run_tests
            deploy_flow_files
            start_flow_wcs
            check_status
            print_info "Full deployment complete!"
            ;;
        *)
            print_error "Unknown command: $COMMAND"
            show_usage
            exit 1
            ;;
    esac
}

# Run main function
main "$@"