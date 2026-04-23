The ReactiveOA is WORKING
___
1. Nuke the `compile_commands.json` thing
2. Add colcon rule `"-DCMAKE_POLICY_DEFAULT_CMP0144=NEW"`
3. Setup stack-like HEAP allocation for DepthCamNode to avoid chasing pointer
4. Setup shared header for alpha_brain