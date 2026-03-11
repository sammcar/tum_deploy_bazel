#!/usr/bin/env python3
import asyncio
import moteus
import moteus_pi3hat
import sys
import time
"""
Motor Position Zeroing Script
This script allows you to set the current position of motors as their new zero reference.
Use this when servos have drifted from their expected zero positions.
"""
# MOTOR CONFIGURATION - Edit this to match your setup
MOTOR_CONFIG = {
    # Currently testing these motors
    1: [1,2,3],
    2: [4, 5, 6],
    3: [7, 8, 9],
    4: [10, 11, 12],
    # Add other motors as needed
    # 1: [1, 2, 3],
    # 2: [4, 5, 6],
    # 4: [10, 11, 12],
}

# LEG CONFIGURATION
LEG_CONFIG = {
    1: [1, 2, 3],
    2: [4, 5, 6],
    3: [7, 8, 9],
    4: [10, 11, 12],
}

# MOTOR DESCRIPTIONS for user guidance
MOTOR_DESCRIPTIONS = {
    # Leg 1
    1: "Motor 1 (Leg 1 Femur)",
    2: "Motor 2 (Leg 1 Tibia)",
    3: "Motor 3 (Leg 1 Shoulder)",
    # Leg 2
    4: "Motor 4 (Leg 2 Femur)",
    5: "Motor 5 (Leg 2 Tibia)",
    6: "Motor 6 (Leg 2 Shoulder)",
    # Leg 3
    7: "Motor 7 (Leg 3 Femur)",
    8: "Motor 8 (Leg 3 Tibia)",
    9: "Motor 9 (Leg 3 Shoulder)",
    # Leg 4
    10: "Motor 10 (Leg 4 Femur)",
    11: "Motor 11 (Leg 4 Tibia)",
    12: "Motor 12 (Leg 4 Shoulder)",
}

class MotorZeroingTool:
    def __init__(self):
        self.transport = None
        self.active_motors = []
        self.controllers = {}
        self.leg_config = LEG_CONFIG
        
        # Get active motors from config
        for bus, motors in MOTOR_CONFIG.items():
            self.active_motors.extend(motors)
        
        print("Motor Zeroing Tool")
        print("=" * 50)
        print(f"Active motors: {self.active_motors}")
        
    async def initialize(self):
        """Initialize transport and controllers"""
        try:
            print("Initializing Pi3HAT transport...")
            self.transport = moteus_pi3hat.Pi3HatRouter(servo_bus_map=MOTOR_CONFIG)
            
            # Create controllers
            for motor_id in self.active_motors:
                self.controllers[motor_id] = moteus.Controller(
                    id=motor_id,
                    transport=self.transport
                )
            
            print("Transport initialized successfully")
            return True
            
        except Exception as e:
            print(f"ERROR: Failed to initialize transport: {e}")
            print("Make sure you're running as root (sudo) and Pi3HAT is connected")
            return False
    
    async def test_communication(self):
        """Test communication with all motors"""
        print("\nTesting motor communication...")
        
        responding_motors = []
        
        for motor_id in self.active_motors:
            try:
                result = await self.transport.cycle([
                    self.controllers[motor_id].make_query()
                ])
                
                if result and len(result) > 0 and result[0].id == motor_id:
                    responding_motors.append(motor_id)
                    description = MOTOR_DESCRIPTIONS.get(motor_id, f"Motor {motor_id}")
                    print(f" ✓ {description} responding")
                else:
                    print(f" ✗ Motor {motor_id} not responding")
                    
            except Exception as e:
                print(f" ✗ Motor {motor_id} error: {e}")
        
        if len(responding_motors) != len(self.active_motors):
            print(f"\nERROR: Only {len(responding_motors)}/{len(self.active_motors)} motors responding")
            print("Check power, connections, and motor IDs")
            return False
        
        print("All motors responding correctly")
        return True
    
    async def show_current_positions(self):
        """Display current positions of all motors"""
        print("\nCurrent motor positions:")
        print("-" * 40)
        
        positions = {}
        
        for motor_id in sorted(self.active_motors):
            try:
                result = await self.transport.cycle([
                    self.controllers[motor_id].make_query()
                ])
                
                if result and len(result) > 0:
                    position = result[0].values.get(moteus.Register.POSITION, 0.0)
                    positions[motor_id] = position
                    description = MOTOR_DESCRIPTIONS.get(motor_id, f"Motor {motor_id}")
                    print(f" {description}: {position:.4f} revolutions ({position*360:.1f} degrees)")
                else:
                    print(f" Motor {motor_id}: No response")
                    positions[motor_id] = 0.0
                    
            except Exception as e:
                print(f" Motor {motor_id}: Error - {e}")
                positions[motor_id] = 0.0
        
        return positions
    
    async def stop_all_motors(self):
        """Stop all motors (disable power)"""
        try:
            commands = [self.controllers[motor_id].make_stop()
                        for motor_id in self.active_motors]
            await self.transport.cycle(commands)
            print("All motors stopped (power disabled)")
            
        except Exception as e:
            print(f"Error stopping motors: {e}")
    
    async def zero_motor(self, motor_id):
        """Zero a single motor"""
        description = MOTOR_DESCRIPTIONS.get(motor_id, f"Motor {motor_id}")
        
        print(f"\nZeroing {description}...")
        print("Position this joint where you want the new ZERO position to be.")
        print("You can manually move the joint now (motors are stopped).")
        
        # Show current position
        try:
            result = await self.transport.cycle([
                self.controllers[motor_id].make_query()
            ])
            
            if result and len(result) > 0:
                current_pos = result[0].values.get(moteus.Register.POSITION, 0.0)
                print(f"Current position: {current_pos:.4f} revolutions ({current_pos*360:.1f} degrees)")
            
        except Exception as e:
            print(f"Error reading position: {e}")
        
        # Wait for user confirmation
        while True:
            user_input = input("Press ENTER to zero this position, 's' to skip, or 'q' to quit: ").strip().lower()
            
            if user_input == 'q':
                return False
            elif user_input == 's':
                return True
            elif user_input == '':
                break
            else:
                print("Invalid input. Use ENTER to zero, 's' to skip, 'q' to quit")
        
        # Perform the zero operation
        try:
            controller = self.controllers[motor_id]
            stream = moteus.Stream(controller)
            
            # Execute rezero command
            await stream.command(b"d rezero")
            await stream.command(b"d cfg-set-output 0")
            await stream.command(b"conf write")
            await asyncio.sleep(0.5)
            
            # Verify the zero was set
            result = await self.transport.cycle([controller.make_query()])
            if result and len(result) > 0:
                new_position = result[0].values.get(moteus.Register.POSITION, 0.0)
                print(f" ✓ {description} zeroed successfully")
                print(f" New position: {new_position:.4f} revolutions ({new_position*360:.1f} degrees)")
                
                if abs(new_position) > 0.01: # 1% of revolution tolerance
                    print(f" WARNING: Position not exactly zero - may have failed")
                    return False
                    
                return True
            else:
                print(f" ERROR: Could not verify zero for {description}")
                return False
                
        except Exception as e:
            print(f" ERROR: Failed to zero {description}: {e}")
            return False
    
    async def zero_all_motors(self):
        """Zero all motors interactively"""
        print("\nStarting interactive zeroing process...")
        print("You will be prompted to position each joint at its desired zero position.")
        
        # Stop all motors first so user can move them manually
        await self.stop_all_motors()
        
        success_count = 0
        
        for motor_id in sorted(self.active_motors):
            if await self.zero_motor(motor_id):
                success_count += 1
            else:
                print("Zeroing process interrupted")
                return False
        
        print(f"\nZeroing complete: {success_count}/{len(self.active_motors)} motors zeroed")
        return success_count == len(self.active_motors)
    
    async def zero_leg(self, leg_id):
        """Zero all motors in a specific leg interactively"""
        if leg_id not in self.leg_config:
            print(f"Invalid leg ID: {leg_id}")
            return False
        
        motors = self.leg_config[leg_id]
        print(f"\nStarting interactive zeroing for Leg {leg_id} (Motors: {motors})...")
        print("You will be prompted to position each joint in the leg at its desired zero position.")
        
        # Stop all motors first so user can move them manually
        await self.stop_all_motors()
        
        success_count = 0
        
        for motor_id in motors:
            if await self.zero_motor(motor_id):
                success_count += 1
            else:
                print("Zeroing process interrupted")
                return False
        
        print(f"\nLeg {leg_id} zeroing complete: {success_count}/{len(motors)} motors zeroed")
        return success_count == len(motors)
    
    async def verify_zeros(self):
        """Verify all motors are at zero position"""
        print("\nVerifying zero positions...")
        
        all_good = True
        
        for motor_id in sorted(self.active_motors):
            try:
                result = await self.transport.cycle([
                    self.controllers[motor_id].make_query()
                ])
                
                if result and len(result) > 0:
                    position = result[0].values.get(moteus.Register.POSITION, 0.0)
                    description = MOTOR_DESCRIPTIONS.get(motor_id, f"Motor {motor_id}")
                    
                    if abs(position) < 0.01: # Within 1% of revolution
                        print(f" ✓ {description}: {position:.4f} rev (GOOD)")
                    else:
                        print(f" ✗ {description}: {position:.4f} rev (NOT ZERO)")
                        all_good = False
                        
            except Exception as e:
                description = MOTOR_DESCRIPTIONS.get(motor_id, f"Motor {motor_id}")
                print(f" ✗ {description}: Error - {e}")
                all_good = False
        
        if all_good:
            print("All motors are properly zeroed!")
        else:
            print("Some motors are not at zero - you may need to re-zero them")
        
        return all_good
    
    async def run_interactive_mode(self):
        """Run the interactive zeroing process"""
        if not await self.initialize():
            return False
        
        if not await self.test_communication():
            return False
        
        while True:
            print("\n" + "="*50)
            print("MOTOR ZEROING MENU")
            print("="*50)
            print("1. Show current positions")
            print("2. Zero all motors (interactive)")
            print("3. Zero full leg (interactive)")
            print("4. Zero individual motor")
            print("5. Verify zero positions")
            print("6. Stop all motors")
            print("7. Exit")
            
            choice = input("\nEnter your choice (1-7): ").strip()
            
            if choice == '1':
                await self.show_current_positions()
                
            elif choice == '2':
                if await self.zero_all_motors():
                    print("All motors zeroed successfully!")
                else:
                    print("Zeroing process failed or was interrupted")
                    
            elif choice == '3':
                print("\nAvailable legs:")
                for leg_id in sorted(self.leg_config.keys()):
                    motors = self.leg_config[leg_id]
                    print(f" {leg_id}: Motors {', '.join(map(str, motors))}")
                
                try:
                    leg_id = int(input("Enter leg number to zero: "))
                    if leg_id in self.leg_config:
                        if await self.zero_leg(leg_id):
                            print(f"Leg {leg_id} zeroed successfully!")
                        else:
                            print(f"Zeroing leg {leg_id} failed or was interrupted")
                    else:
                        print("Invalid leg number")
                except ValueError:
                    print("Invalid input - enter a number")
                    
            elif choice == '4':
                await self.show_current_positions()
                print("\nAvailable motors:")
                for motor_id in sorted(self.active_motors):
                    description = MOTOR_DESCRIPTIONS.get(motor_id, f"Motor {motor_id}")
                    print(f" {motor_id}: {description}")
                
                try:
                    motor_id = int(input("Enter motor ID to zero: "))
                    if motor_id in self.active_motors:
                        await self.stop_all_motors()
                        await self.zero_motor(motor_id)
                    else:
                        print("Invalid motor ID")
                except ValueError:
                    print("Invalid input - enter a number")
                    
            elif choice == '5':
                await self.verify_zeros()
                
            elif choice == '6':
                await self.stop_all_motors()
                
            elif choice == '7':
                await self.stop_all_motors()
                print("Exiting...")
                break
                
            else:
                print("Invalid choice")

async def main():
    tool = MotorZeroingTool()
    
    print("IMPORTANT: Position your motors in their desired ZERO positions")
    print("before running this script. The zero position is typically:")
    print("- Shoulder: Neutral (leg straight out from body)")
    print("- Femur: Vertical down from hip")
    print("- Tibia: Vertical down from knee (leg fully extended)")
    print()
    
    try:
        await tool.run_interactive_mode()
        
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        await tool.stop_all_motors()
        
    except Exception as e:
        print(f"Unexpected error: {e}")
        if tool.transport:
            await tool.stop_all_motors()

if __name__ == '__main__':
    # Check if running as root
    import os
    if os.geteuid() != 0:
        print("ERROR: This script must be run as root")
        print("Usage: sudo python3 zero_motors.py")
        sys.exit(1)
    
    asyncio.run(main())
