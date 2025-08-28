"""
TAFL Command Line Interface
TAFL 命令列介面工具
"""
import argparse
import sys
import json
import yaml
import asyncio
from pathlib import Path
from typing import Optional
from .parser import TAFLParser, TAFLParseError
from .validator import TAFLValidator, TAFLValidationError
from .converter import TAFLConverter
from .executor import TAFLExecutor


def main():
    """Main CLI entry point / CLI 主入口"""
    parser = argparse.ArgumentParser(
        description='TAFL - Task Automation Flow Language CLI',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  tafl validate flow.yaml          # Validate a TAFL file
  tafl convert flow.yaml -o v2.yaml  # Convert to Linear Flow v2
  tafl info flow.yaml              # Show flow information
  tafl run flow.yaml --dry-run     # Parse without executing
        """
    )
    parser.add_argument('--version', action='version', version='%(prog)s 1.0.0')
    
    subparsers = parser.add_subparsers(dest='command', help='Available commands')
    
    # Validate command
    validate_parser = subparsers.add_parser('validate', help='Validate TAFL file')
    validate_parser.add_argument('file', help='TAFL YAML file to validate')
    validate_parser.add_argument('-v', '--verbose', action='store_true', 
                                help='Show detailed validation info')
    
    # Convert command
    convert_parser = subparsers.add_parser('convert', help='Convert TAFL to Linear Flow v2')
    convert_parser.add_argument('file', help='TAFL YAML file to convert')
    convert_parser.add_argument('-o', '--output', help='Output file path')
    convert_parser.add_argument('--pretty', action='store_true',
                               help='Pretty print output')
    
    # Run command
    run_parser = subparsers.add_parser('run', help='Execute TAFL flow')
    run_parser.add_argument('file', help='TAFL YAML file to execute')
    run_parser.add_argument('--context', help='Initial context JSON')
    run_parser.add_argument('--dry-run', action='store_true', 
                           help='Parse and validate without executing')
    
    # Info command
    info_parser = subparsers.add_parser('info', help='Show TAFL file information')
    info_parser.add_argument('file', help='TAFL YAML file')
    info_parser.add_argument('--json', action='store_true',
                            help='Output as JSON')
    
    args = parser.parse_args()
    
    if not args.command:
        parser.print_help()
        return 0
    
    try:
        if args.command == 'validate':
            return validate_command(args)
        elif args.command == 'convert':
            return convert_command(args)
        elif args.command == 'run':
            return run_command(args)
        elif args.command == 'info':
            return info_command(args)
    except KeyboardInterrupt:
        print("\nOperation cancelled by user")
        return 130
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1


def validate_command(args) -> int:
    """Validate TAFL file / 驗證 TAFL 檔案"""
    try:
        parser = TAFLParser()
        validator = TAFLValidator()
        
        file_path = Path(args.file)
        if not file_path.exists():
            print(f"Error: File not found: {args.file}", file=sys.stderr)
            return 1
        
        with open(file_path, 'r', encoding='utf-8') as f:
            yaml_content = f.read()
        
        # Parse the file
        try:
            program = parser.parse_string(yaml_content)
        except TAFLParseError as e:
            print(f"❌ Parse error: {e}", file=sys.stderr)
            return 1
        
        # Validate the program
        is_valid = validator.validate(program)
        
        # Show errors and warnings if verbose
        if args.verbose or not is_valid:
            for error in validator.get_errors():
                print(f"ERROR: {error}", file=sys.stderr)
            for warning in validator.get_warnings():
                print(f"WARNING: {warning}", file=sys.stderr)
        
        if is_valid:
            print(f"✅ {args.file} is valid TAFL")
            if not args.verbose and validator.get_warnings():
                print(f"   ({len(validator.get_warnings())} warnings - use -v to see details)")
            return 0
        else:
            print(f"❌ {args.file} has {len(validator.get_errors())} validation errors")
            return 1
            
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1


def convert_command(args) -> int:
    """Convert TAFL to Linear Flow v2 / 轉換 TAFL 為 Linear Flow v2"""
    try:
        parser = TAFLParser()
        converter = TAFLConverter()
        
        file_path = Path(args.file)
        if not file_path.exists():
            print(f"Error: File not found: {args.file}", file=sys.stderr)
            return 1
        
        with open(file_path, 'r', encoding='utf-8') as f:
            yaml_content = f.read()
        
        # Parse the TAFL file
        try:
            program = parser.parse_string(yaml_content)
        except TAFLParseError as e:
            print(f"Parse error: {e}", file=sys.stderr)
            return 1
        
        # Convert to v2
        v2_dict = converter.convert(program)
        
        # Format output
        if args.pretty:
            v2_yaml = yaml.dump(v2_dict, default_flow_style=False, 
                              allow_unicode=True, sort_keys=False,
                              width=100, indent=2)
        else:
            v2_yaml = yaml.dump(v2_dict, default_flow_style=False, 
                              allow_unicode=True, sort_keys=False)
        
        # Output result
        if args.output:
            output_path = Path(args.output)
            output_path.parent.mkdir(parents=True, exist_ok=True)
            with open(output_path, 'w', encoding='utf-8') as f:
                f.write(v2_yaml)
            print(f"✅ Converted to {args.output}")
        else:
            print(v2_yaml)
        
        return 0
        
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1


def run_command(args) -> int:
    """Execute TAFL flow / 執行 TAFL 流程"""
    try:
        parser = TAFLParser()
        
        file_path = Path(args.file)
        if not file_path.exists():
            print(f"Error: File not found: {args.file}", file=sys.stderr)
            return 1
        
        with open(file_path, 'r', encoding='utf-8') as f:
            yaml_content = f.read()
        
        # Parse the file
        try:
            program = parser.parse_string(yaml_content)
        except TAFLParseError as e:
            print(f"Parse error: {e}", file=sys.stderr)
            return 1
        
        # Load initial context if provided
        initial_context = {}
        if args.context:
            try:
                initial_context = json.loads(args.context)
            except json.JSONDecodeError as e:
                print(f"Error parsing context JSON: {e}", file=sys.stderr)
                return 1
        
        if args.dry_run:
            # Dry run - just validate
            validator = TAFLValidator()
            is_valid = validator.validate(program)
            
            print(f"Flow: {program.metadata.name} (ID: {program.metadata.id})")
            print(f"Statements: {len(program.flow)}")
            print(f"Variables: {len(program.variables)}")
            
            if is_valid:
                print("✅ Flow is valid and ready to execute")
            else:
                print(f"❌ Flow has {len(validator.get_errors())} errors")
                for error in validator.get_errors():
                    print(f"  - {error}")
            
            return 0 if is_valid else 1
        else:
            # Actual execution
            print("Note: TAFL execution requires integration with flow_wcs functions")
            print("      This feature is not yet available in standalone mode")
            return 1
        
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1


def info_command(args) -> int:
    """Show TAFL file information / 顯示 TAFL 檔案資訊"""
    try:
        parser = TAFLParser()
        
        file_path = Path(args.file)
        if not file_path.exists():
            print(f"Error: File not found: {args.file}", file=sys.stderr)
            return 1
        
        with open(file_path, 'r', encoding='utf-8') as f:
            yaml_content = f.read()
        
        # Parse the file
        try:
            program = parser.parse_string(yaml_content)
        except TAFLParseError as e:
            print(f"Parse error: {e}", file=sys.stderr)
            return 1
        
        # Collect information
        info = {
            'id': program.metadata.id,
            'name': program.metadata.name,
            'version': program.metadata.version,
            'description': program.metadata.description or 'N/A',
            'author': program.metadata.author or 'N/A',
            'tags': program.metadata.tags,
            'statements': len(program.flow),
            'variables': len(program.variables),
            'settings': {
                'timeout': program.settings.timeout,
                'retry_on_failure': program.settings.retry_on_failure,
                'max_retries': program.settings.max_retries,
                'parallel_execution': program.settings.parallel_execution,
                'log_level': program.settings.log_level,
            }
        }
        
        # Count statement types
        statement_types = {}
        for stmt in program.flow:
            stmt_type = type(stmt).__name__.replace('Statement', '')
            statement_types[stmt_type] = statement_types.get(stmt_type, 0) + 1
        info['statement_types'] = statement_types
        
        # Output format
        if args.json:
            print(json.dumps(info, indent=2, ensure_ascii=False))
        else:
            print(f"Flow ID: {info['id']}")
            print(f"Flow Name: {info['name']}")
            print(f"Version: {info['version']}")
            print(f"Description: {info['description']}")
            print(f"Author: {info['author']}")
            if info['tags']:
                print(f"Tags: {', '.join(info['tags'])}")
            print(f"\nStatistics:")
            print(f"  Total Statements: {info['statements']}")
            print(f"  Variables: {info['variables']}")
            
            if statement_types:
                print(f"\nStatement Types:")
                for stmt_type, count in sorted(statement_types.items()):
                    print(f"  {stmt_type}: {count}")
            
            print(f"\nSettings:")
            for key, value in info['settings'].items():
                print(f"  {key}: {value}")
        
        return 0
        
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())