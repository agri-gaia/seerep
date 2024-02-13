import json
from pathlib import Path

DATUMARO_JSON_PATH = "/seerep/seerep-data/datumaro_test/annotations/default.json"


class DatumaroImporter:
    def __init__(self, datumaro_path: str):
        self.datumaro_path = Path(datumaro_path)
        if not self.valid_json_path():
            raise ValueError('Invalid datumaro JSON path')
        self.labels = None

    def valid_json_path(self) -> bool:
        return self.datumaro_path.exists() and self.datumaro_path.is_file() and self.datumaro_path.suffix == '.json'

    # TODO: this loads the entire json file into memory, bad
    def iterate_json(self) -> None:
        with open(self.datumaro_path, 'r') as f:
            data = json.load(f)
            self.label = {
                item['name']: {k: v for k, v in item.items() if k != 'name'}
                for item in data['categories']['label']['labels']
            }


if __name__ == '__main__':
    di = DatumaroImporter(DATUMARO_JSON_PATH)
    di.iterate_json()
