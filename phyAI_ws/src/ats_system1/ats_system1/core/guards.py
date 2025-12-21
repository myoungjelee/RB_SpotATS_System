def eval_guard(expr: str, sym: dict) -> bool:
    safe = expr
    for k, v in sym.items():
        safe = safe.replace(k, str(v).lower() if isinstance(v, bool) else str(v))
    safe = safe.replace("&&", " and ").replace("||", " or ").replace("!", " not ")
    return bool(eval(safe, {"__builtins__": {}}, {}))
