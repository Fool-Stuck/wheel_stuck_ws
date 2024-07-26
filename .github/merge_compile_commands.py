import json
import os


def merge_compile_commands(output_file):
    """Merge all compile_commands.json files into one."""
    compile_commands = []
    for root, dirs, files in os.walk("."):
        if "compile_commands.json" in files:
            with open(os.path.join(root, "compile_commands.json"), "r") as f:
                compile_commands.extend(json.load(f))
    with open(output_file, "w") as f:
        json.dump(compile_commands, f, indent=2)


if __name__ == "__main__":
    merge_compile_commands("compile_commands.json")
