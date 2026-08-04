def AddToLog(text, fileName):
    log_file = open(f"{fileName}.txt", "a+")
    log_file.write(text + "\n")
    log_file.close()

    print(text)


def AddTable(fileName, title, headers, rows):
    """Print an aligned table and append it to <fileName>.txt.

    rows: list of rows, each row a list of strings (already formatted).
    """
    if not rows:
        return

    widths = [len(str(h)) for h in headers]
    for r in rows:
        for i, cell in enumerate(r):
            widths[i] = max(widths[i], len(str(cell)))

    fmt = "  ".join(f"{{:<{w}}}" for w in widths)
    total = sum(widths) + 2 * (len(headers) - 1)

    lines = [title, "-" * total, fmt.format(*headers), "-" * total]
    for r in rows:
        lines.append(fmt.format(*[str(c) for c in r]))
    lines.append("")

    text = "\n".join(lines)
    log_file = open(f"{fileName}.txt", "a+")
    log_file.write(text + "\n")
    log_file.close()

    print(text)
