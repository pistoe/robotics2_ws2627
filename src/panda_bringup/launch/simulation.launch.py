from panda_bringup.launches import generate


def generate_launch_description():
    return generate("simulation", student=True)
