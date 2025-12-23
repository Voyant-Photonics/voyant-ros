#!/usr/bin/env python3
# Copyright (c) 2024-2025 Voyant Photonics, Inc.
#
# This example code is licensed under the MIT License.
# See the LICENSE file in the repository root for full license text.

"""
Build Foxglove configuration JSON from template and TypeScript source files.

This script reads a template JSON file with placeholder userNodes and replaces
the sourceCode fields with the contents of the corresponding .ts files.

Usage:
    python build_foxglove_config.py [--scripts-dir DIR] [--template FILE] [--output FILE]

The template JSON should have userNodes entries like:
    "userNodes": {
        "869d30d4-...": {
            "sourceCode": "{{snr_clr_add_fields.ts}}",
            "name": "snr_clr_add_fields.ts"
        },
        ...
    }

The script will replace {{filename.ts}} with the contents of scripts-dir/filename.ts
"""

import argparse
import json
import re
import sys
from pathlib import Path


def read_script_file(scripts_dir: Path, filename: str) -> str:
    """Read a TypeScript file and return its contents."""
    script_path = scripts_dir / filename

    if not script_path.exists():
        raise FileNotFoundError(f"Script file not found: {script_path}")

    with open(script_path, 'r', encoding='utf-8') as f:
        return f.read()


def build_config(template_path: Path, scripts_dir: Path, output_path: Path) -> bool:
    """
    Build the final Foxglove config JSON.

    Returns True on success, False on failure.
    """
    # Read template
    print(f"Reading template: {template_path}")
    with open(template_path, 'r', encoding='utf-8') as f:
        config = json.load(f)

    user_nodes = config.get('userNodes', {})

    if not user_nodes:
        print("WARNING: No userNodes found in template")
        return True

    # Process each user node
    scripts_replaced = 0
    for node_id, node_data in user_nodes.items():
        source_code = node_data.get('sourceCode', '')

        # Check if sourceCode is a placeholder like {{filename.ts}}
        match = re.match(r'^\{\{(.+\.ts)\}\}$', source_code.strip())

        if match:
            filename = match.group(1)
            try:
                script_content = read_script_file(scripts_dir, filename)
                node_data['sourceCode'] = script_content
                print(f"Embedded: {filename} ({len(script_content)} bytes)")
                scripts_replaced += 1
            except FileNotFoundError as e:
                print(f"ERROR: {e}")
                return False
        else:
            # Already has inline content or empty
            name = node_data.get('name', node_id)
            if source_code:
                print(f"WARNING: Skipped {name}: sourceCode is not a placeholder")
            else:
                print(f"WARNING: Skipped {name}: sourceCode is empty")

    # Write output
    print(f"Writing output: {output_path}")

    # Add generation comment (we'll prepend it as a note since JSON doesn't support comments)
    # Instead, we just write the JSON and rely on the header in the file

    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(config, f, indent=2)

    print(f"Done! Replaced {scripts_replaced} script(s)")
    return True


def main():
    parser = argparse.ArgumentParser(
        description='Build Foxglove config JSON from template and TypeScript files',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Using defaults (looks for files in ./config/)
    python build_foxglove_config.py

    # Specify all paths
    python config/build_foxglove_config.py \
        --template config/voyant_foxglove_cfg.template.json \
        --scripts-dir config/foxglove_user_scripts \
        --output config/voyant_foxglove_cfg.json
        """
    )

    parser.add_argument(
        '--template', '-t',
        type=Path,
        default=Path('config/voyant_foxglove_cfg.template.json'),
        help='Path to template JSON file (default: config/voyant_foxglove_cfg.template.json)'
    )

    parser.add_argument(
        '--scripts-dir', '-s',
        type=Path,
        default=Path('config/foxglove_user_scripts'),
        help='Directory containing .ts script files (default: config/foxglove_user_scripts)'
    )

    parser.add_argument(
        '--output', '-o',
        type=Path,
        default=Path('config/voyant_foxglove_cfg.json'),
        help='Output JSON file path (default: config/voyant_foxglove_cfg.json)'
    )

    args = parser.parse_args()

    # Validate inputs
    if not args.template.exists():
        print(f"ERROR: Template file not found: {args.template}")
        sys.exit(1)

    if not args.scripts_dir.exists():
        print(f"ERROR: Scripts directory not found: {args.scripts_dir}")
        sys.exit(1)

    # Build config
    success = build_config(args.template, args.scripts_dir, args.output)

    if not success:
        sys.exit(1)


if __name__ == '__main__':
    main()
