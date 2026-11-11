"""Register numbered policies in a session's private simulation config."""

from pathlib import Path
import re

from tools.virtual_gamepad.action_catalog import (
    READY_MOTIONS, load_catalog, motion_name, task_keys,
)


def numbered_task_graph(original: str, actions: list[dict]) -> str:
    """Preserve existing tasks and extend only their explicit entry edges."""
    names = [motion_name(action["id"]) for action in actions]
    headers = list(re.finditer(r"(?m)^  - motion: ([A-Za-z0-9_]+)[ \t]*(?:#[^\r\n]*)?$", original))
    existing = [match[1] for match in headers]
    if len(existing) != len(set(existing)) or not READY_MOTIONS.issubset(existing):
        raise ValueError("Numbered actions require unique existing PD and walking tasks")
    if set(existing).intersection(names):
        raise ValueError("Numbered action task already exists")
    result = original[:headers[0].start()]
    for index, match in enumerate(headers):
        end = headers[index + 1].start() if index + 1 < len(headers) else len(original)
        block = original[match.start():end]
        if match[1] in READY_MOTIONS:
            block, count = re.subn(
                r"(?m)^(    manual_transition:[ \t]*(?:#[^\r\n]*)?\n)",
                lambda item: item[1] + "".join(f"      - {name}\n" for name in names),
                block,
            )
            if count != 1:
                raise ValueError(f"Expected one manual transition list for {match[1]}")
        result += block
    for action, name in zip(actions, names):
        result += (
            f"\n  - motion: {name}\n"
            "    period: 0.02\n"
            "    runner:\n"
            "      - name: rl_supermimic_runner\n"
            "        enabled: true\n"
            f"        param_tag: {name}\n"
            "    manual_transition:\n"
            "      - passive\n"
            + "".join(f"      - {target}\n" for target in sorted(READY_MOTIONS))
            + "    auto_transition: walk_leo\n"
            + f"    key: [{', '.join(task_keys(action['id']))}]\n"
        )
    return result


def numbered_mode(original: str, actions: list[dict]) -> str:
    """Add tags only within mode.sim; leave the hardware configuration intact."""
    headers = list(re.finditer(r"(?m)^  sim:[ \t]*(?:#[^\r\n]*)?$", original))
    if len(headers) != 1:
        raise ValueError("Expected one sim mode for numbered actions")
    start = headers[0].end()
    next_mode = re.search(r"(?m)^  [A-Za-z0-9_]+:", original[start:])
    end = start + next_mode.start() if next_mode else len(original)
    section = original[start:end]
    if re.search(r"(?m)^    - tag: test_action_", section):
        raise ValueError("Numbered action parameter tag already exists")
    section, count = re.subn(
        r"(?m)^(    - tag: motion_task[ \t]*(?:#[^\r\n]*)?\n"
        r"      scope:[ \t]*)task_motion/default([ \t]*(?:#[^\r\n]*)?)$",
        r"\g<1>task_motion/numbered_actions\2", section,
    )
    if count != 1:
        raise ValueError("Numbered actions require motion_task scope task_motion/default")
    if not section.endswith("\n"):
        section += "\n"
    for action in actions:
        section += (
            f"    - tag: {motion_name(action['id'])}\n"
            f"      scope: numbered_actions/{action['package']}/default\n"
        )
    return original[:start] + section + original[end:]


def configure_numbered_actions(config: Path) -> Path | None:
    """Return the UI catalog, or None when no personal actions are configured."""
    catalog_path = config / "numbered_actions/catalog.json"
    if not catalog_path.exists():
        return None
    catalog = load_catalog(catalog_path, check_assets=True)
    actions = catalog["actions"]
    if not actions:
        return None
    mode_path = config / "mode.yaml"
    task_path = config / "task_motion/default.yaml"
    output_path = config / "task_motion/numbered_actions.yaml"
    if (mode_path.is_symlink() or task_path.is_symlink() or output_path.is_symlink()
            or (config / "task_motion").is_symlink()):
        raise ValueError("Numbered actions require private regular mode and task config files")
    mode = numbered_mode(mode_path.read_text(encoding="utf-8"), actions)
    tasks = numbered_task_graph(task_path.read_text(encoding="utf-8"), actions)
    output_path.write_text(tasks, encoding="utf-8")
    mode_path.write_text(mode, encoding="utf-8")
    return catalog_path
