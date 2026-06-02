file_name = "float_ui.html"
pre_content = "const char index_html[] PROGMEM = R\"rawliteral("
post_content = ")rawliteral\";"

with open(file_name, "r") as f:
    content = f.read()


with open("../include/html_rendered.h", "w+") as f:
    f.write(pre_content)
    f.write(content)
    f.write(post_content)