# vendor/nxp-opensource/kernel_imx/logo_rules.bzl

def logo_gen(name, fmt, ext):
    """in drivers/video/logo out <name>.c"""
    native.genrule(
        name   = name + "_c",
        srcs   = ["drivers/video/logo/%s.%s" % (name, ext)],
        tools  = [":pnmtologo"],
        outs   = ["drivers/video/logo/%s.c" % name],
        cmd    = "$(location :pnmtologo) -t %s -n %s -a \"\" -o $@ $<" % (fmt, name),
    )
