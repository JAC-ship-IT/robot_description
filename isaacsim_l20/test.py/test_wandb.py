import wandb
try:
    wandb.init(project="linker-hand", entity="wang_jie2333")
    wandb.log({"test": 1})
    wandb.log({"test": 2})
    wandb.log({"test": 3})
    wandb.log({"test": 4})
    wandb.log({"test": 5})
    wandb.log({"test": 6})
    wandb.log({"test": 7})
    wandb.log({"test": 8})
    wandb.log({"test": 9})
    wandb.log({"test": 10})
except Exception as e:
    print(e)
print("Wandb initialized!!!!!!!!!!!!!")