Import("env")

print("Current CLI targets", COMMAND_LINE_TARGETS)
print("Current Build targets", BUILD_TARGETS)

def after_upload(source, target, env):
    env.Execute("pio run --target uploadfs")

env.AddPostAction("upload", after_upload)