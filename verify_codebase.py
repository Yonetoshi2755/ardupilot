#!/usr/bin/env python3
"""
Comprehensive verification script for the Bridge Inspector codebase
"""

import os
import sys
import importlib.util
import ast
from pathlib import Path

# Color codes for output
GREEN = '\033[92m'
YELLOW = '\033[93m'
RED = '\033[91m'
RESET = '\033[0m'

def check_syntax(file_path):
    """Check Python syntax by parsing the AST"""
    try:
        with open(file_path, 'r') as f:
            ast.parse(f.read())
        return True, None
    except SyntaxError as e:
        return False, str(e)

def extract_imports(file_path):
    """Extract all imports from a Python file"""
    imports = set()
    try:
        with open(file_path, 'r') as f:
            tree = ast.parse(f.read())
        
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for alias in node.names:
                    imports.add(alias.name.split('.')[0])
            elif isinstance(node, ast.ImportFrom):
                if node.module:
                    imports.add(node.module.split('.')[0])
    except:
        pass
    
    return imports

def check_module_available(module_name):
    """Check if a module can be imported"""
    spec = importlib.util.find_spec(module_name)
    return spec is not None

def analyze_function_calls(file_path):
    """Analyze function calls to check for potential issues"""
    issues = []
    try:
        with open(file_path, 'r') as f:
            tree = ast.parse(f.read())
        
        # Check for common issues
        for node in ast.walk(tree):
            # Check for undefined variables in function calls
            if isinstance(node, ast.Call):
                if isinstance(node.func, ast.Name):
                    # Basic check for some known issues
                    if node.func.id == 'motors_armed' and not any(isinstance(n, ast.Attribute) for n in ast.walk(node)):
                        issues.append(f"Line {node.lineno}: 'motors_armed' might need to be called as a method")
                        
    except:
        pass
    
    return issues

def verify_file(file_path):
    """Verify a single Python file"""
    print(f"\nChecking: {file_path}")
    
    # Check syntax
    syntax_ok, error = check_syntax(file_path)
    if not syntax_ok:
        print(f"  {RED}✗ Syntax Error:{RESET} {error}")
        return False
    else:
        print(f"  {GREEN}✓ Syntax OK{RESET}")
    
    # Extract and check imports
    imports = extract_imports(file_path)
    missing_imports = []
    
    # Standard library and known available modules
    stdlib_modules = {
        'asyncio', 'time', 'math', 'json', 'datetime', 'typing', 'dataclasses',
        'enum', 'logging', 'os', 'sys', 'csv', 'base64', 'io', 'pathlib',
        'warnings', 'collections', 'functools', 'itertools', 'subprocess'
    }
    
    # Third-party modules that should be installed
    required_modules = {
        'pymavlink', 'numpy', 'cv2', 'torch', 'torchvision', 'PIL',
        'pandas', 'matplotlib', 'folium', 'jinja2', 'yaml'
    }
    
    for imp in imports:
        if imp not in stdlib_modules and imp not in required_modules:
            if imp == 'src' or imp == 'scripts':
                continue  # Local imports
            if not check_module_available(imp):
                missing_imports.append(imp)
    
    if missing_imports:
        print(f"  {YELLOW}⚠ Missing imports:{RESET} {', '.join(missing_imports)}")
    else:
        print(f"  {GREEN}✓ All imports available{RESET}")
    
    # Analyze for potential issues
    issues = analyze_function_calls(file_path)
    if issues:
        print(f"  {YELLOW}⚠ Potential issues:{RESET}")
        for issue in issues:
            print(f"    - {issue}")
    
    return syntax_ok and not missing_imports

def check_file_connections():
    """Check if files are properly connected"""
    print("\n" + "="*60)
    print("Checking file connections and dependencies")
    print("="*60)
    
    # Check if main entry points exist
    entry_points = [
        "scripts/run_inspection.py",
        "examples/example_golden_gate.py"
    ]
    
    for entry in entry_points:
        if os.path.exists(entry):
            print(f"{GREEN}✓{RESET} Entry point exists: {entry}")
        else:
            print(f"{RED}✗{RESET} Entry point missing: {entry}")
    
    # Check if all imported modules exist
    src_files = list(Path('src').glob('*.py'))
    module_names = {f.stem for f in src_files}
    
    print(f"\nFound modules in src/: {', '.join(module_names)}")
    
    # Check cross-references
    for file in src_files:
        content = file.read_text()
        for module in module_names:
            if module != file.stem and f"from src.{module}" in content:
                print(f"{GREEN}✓{RESET} {file.stem} imports {module}")

def check_specific_issues():
    """Check for specific known issues"""
    print("\n" + "="*60)
    print("Checking for specific issues")
    print("="*60)
    
    issues_found = False
    
    # Check inspection_controller.py for motors_armed issue
    controller_path = "src/inspection_controller.py"
    if os.path.exists(controller_path):
        with open(controller_path, 'r') as f:
            content = f.read()
            
        # Line 114 issue
        if "self.vehicle.motors_armed()" in content:
            print(f"{YELLOW}⚠{RESET} Line 114 in inspection_controller.py:")
            print(f"  'motors_armed()' should likely be 'motors_armed' (property, not method)")
            print(f"  Or it should be a proper MAVLink check")
            issues_found = True
    
    # Check for proper MAVLink usage
    if os.path.exists(controller_path):
        with open(controller_path, 'r') as f:
            lines = f.readlines()
        
        for i, line in enumerate(lines, 1):
            if "flightmode ==" in line and "self.vehicle.flightmode" in line:
                print(f"{YELLOW}⚠{RESET} Line {i} in inspection_controller.py:")
                print(f"  MAVLink flight mode check might need adjustment")
                print(f"  Consider using recv_match with HEARTBEAT message")
                issues_found = True
    
    if not issues_found:
        print(f"{GREEN}✓{RESET} No specific known issues found")
    
    return not issues_found

def check_config_files():
    """Check configuration files"""
    print("\n" + "="*60)
    print("Checking configuration files")
    print("="*60)
    
    config_files = [
        "config/mission_config.yaml",
        "requirements.txt",
        "README.md"
    ]
    
    for config in config_files:
        if os.path.exists(config):
            print(f"{GREEN}✓{RESET} Config file exists: {config}")
        else:
            print(f"{RED}✗{RESET} Config file missing: {config}")

def main():
    """Main verification function"""
    print("Bridge Inspector Codebase Verification")
    print("="*60)
    
    # Get all Python files
    python_files = []
    for pattern in ['src/*.py', 'scripts/*.py', 'examples/*.py']:
        python_files.extend(Path('.').glob(pattern))
    
    print(f"Found {len(python_files)} Python files to check")
    
    # Verify each file
    all_ok = True
    for file in python_files:
        if not verify_file(file):
            all_ok = False
    
    # Check file connections
    check_file_connections()
    
    # Check specific issues
    specific_ok = check_specific_issues()
    
    # Check config files
    check_config_files()
    
    # Summary
    print("\n" + "="*60)
    print("SUMMARY")
    print("="*60)
    
    if all_ok and specific_ok:
        print(f"{GREEN}✓ All checks passed!{RESET}")
        print("\nThe codebase appears to be properly structured.")
        print("Note: Some third-party dependencies need to be installed:")
        print("  pip install -r requirements.txt")
    else:
        print(f"{RED}✗ Some issues found{RESET}")
        print("\nRecommended fixes:")
        print("1. Fix the motors_armed() method call in inspection_controller.py")
        print("2. Review MAVLink usage for proper attribute access")
        print("3. Install missing dependencies")
    
    print("\nMain entry points:")
    print("  - scripts/run_inspection.py - Main inspection runner")
    print("  - examples/example_golden_gate.py - Example usage")

if __name__ == "__main__":
    main()