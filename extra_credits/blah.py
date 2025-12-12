with open("extra_credits/output.mem", "r") as f:
    content = f.readlines()
    content = [c.strip() for c in content if c != "\n"]

output_code = ""
for code in content:
    output_code = output_code + f"{code[6:]}\n" + f"{code[4:6]}\n" + f"{code[2:4]}\n"+  f"{code[:2]}\n"

with open("extra_credits/output.mem", "w") as f:
    f.write(output_code)