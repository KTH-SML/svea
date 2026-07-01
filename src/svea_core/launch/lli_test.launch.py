#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this

@launch_this
def main(
    name: str = 'self',
):
    bl = BetterLaunch()

    bl.include("svea_core", "svea.launch.py",
               name=name,
               is_sim=False,
               use_localization=False)
    
    bl.node("svea_core", "lli_test.py",
            name="lli_test",)
