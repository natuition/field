from dotenv import set_key, get_key
from pathlib import Path

class EnvironnementConfig:
    env_file_path = Path("./.env")
    env_file_path.touch(mode=0o600, exist_ok=True)

    @classmethod
    def NATUITION_CHECKLIST(cls, value=None):
        if value is not None:
            set_key(dotenv_path=cls.env_file_path, key_to_set="NATUITION_CHECKLIST", value_to_set=str(value))
        else:
            value = get_key(dotenv_path=cls.env_file_path, key_to_get="NATUITION_CHECKLIST")
            if value is None:
                return None
            return eval(value)

def test_environnement_config():
    print(EnvironnementConfig.NATUITION_CHECKLIST())
    EnvironnementConfig.NATUITION_CHECKLIST(True)
    print(EnvironnementConfig.NATUITION_CHECKLIST())
    EnvironnementConfig.NATUITION_CHECKLIST(False)
    print(EnvironnementConfig.NATUITION_CHECKLIST())
    
if "__main__" == __name__:
    test_environnement_config()