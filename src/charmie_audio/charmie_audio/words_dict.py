# this variable is just to help to better visualise for debug purposes
max_number_of_chars_of_keys = 16

"""
names_dict = {
    "Adel":     ["adel", "adele", "adam", "avel"],
    "Angel":    ["angel", "anjo", "ahil", "anjou", "anjal", "hong kong", "zangal", "enzo" ], # removed "hell" because of possible "hello" people may say
    "Axel":     ["axel", "oxford", "aksal", "hotel", "axo", "oxel, oxyl", "axil", "unhail", "oxo", "axle", "oxley", "excellent"],
    "Charlie":  ["charlie", "shire lee", "sean", "shoutily", "chorley", "cherry"],
    "Jane":     ["jane", "jan", "jain", "jame", "jani", "jainie"],
    "John":     ["john", "jhong", "joe", "jopne", "joan", "chiang"],
    "Jules":    ["jules", "jews", "drews", "jewels", "julie"],
    "Morgan":   ["morgan", "modern", "mardigan", "morgana", "mokkan", "maud", "modogan"],
    "Paris":    ["paris", "berries", "ferries", "parish", "pari", "hari", "barry", "patrice", "paix"],
    "Robin":    ["robin", "harbin", "robyn", "hobby", "hobean", "hohvin", "robon", "corbin", "haubin", "horven"],
    "Simone":   ["simone", "simon", "simo", "siman"]
}
"""

names_dict = {
    "Kai":      ["kai", "kite", "guy", " kei ", " kay ", "kate", " k "],
    "Noah":     ["noah", "no one", "noha"],
    "Unique":   ["unique", "vinik", "yoonique", "eunik", " unic ", "henic", "eunich"],
    "Luca":     ["luca", "luka", "loga", "mocha"],
    "Evelyn":   ["evelyn", "evelin", "hevelin", "heavily", " evan "],
    "Jayden":   ["jayden", "j don", "jaden", "raven", "haydn", " haidan"],
    "James":    ["james", "denise", "hamis", "hamiz"],
    "Riley":    ["riley", "really", "dryland", "rylan", "rylu"],
    "Harper":   ["harper", "khadipah", "harpur", "arper", "arthur", "harpe"],
    "Quinn":    ["quinn", "clem", "queen", "green"],
    "Avery":    ["avery", "every", "henry", "everett"],
    "Remi":     ["remi", "renee", " rani ", " rami ", "remy", "ray me", "rene", "hemi", " vami ", "raimi"],
    "River":    ["river", "rever", "reaver", "rida", "rivel", "peever", " recer "], # david?
    "Atlas":    ["atlas", "atlash", "atles", "art loz", "akhlish", "ad-lose", "after", " atash ", "akish", "arthlash", "akash", "akulosh", "akulish", "ateles", " asla ", "aatish", " athos "]
}


drinks_dict = {
    "Red_Wine":       ["red", "wine", "edvin", "royce", "white", "ready", "headwind"],
    "Juice_Pack":     ["pack", "gispeck", "druspak", "juspek", "swissbag", "speck", "just play", "packing", "becky",\
                       "bagging", "pachy", "peky", "paki", "peggy", "bek", "bake", "beck", "back", "speak", "bank", "juice bag", "drew spec",\
                       "peck", "druspek", "drip spec", "juicy pek", "juice but", "respect", " juspe ", "jeez park"],
    "Cola":           [" cola", "coke", "coca", "coconut", "cook", "cok", "color", "call", "gog", "koch", "kullum", "goog", "cold", "kohler", "kala", "gola"],
    "Tropical_Juice": ["tropical", "e-copic", "trophy dungeons", "tropi"],
    "Milk":           ["milk", "muke", "mew", "milburn", "mio", "mieuki", "mute", "mille", "mieok", "milton"],
	"Iced_Tea":       [" ice", "tea", "i-st", "stick", "i see", "i still", "i stick", "ice cream", "icedy",\
                       "icedi", "istie", "isti", "isalutti", "hasty", "i sati", "i see", "istia", "i steve", "i stay",\
                       "an i.c.", "anasti", "aussie", "aesti", "esti", "nice d", " icy ", "nice t", "oysti"],
    "Orange_Juice":   ["orange", "orang", "orangous", "orangels", "foreign", "autumn", "orton", "orandris", "our enjour", "northern juice", "orton julius",\
    # "Orange_Juice":   ["orange juice", "orangous", "orangels", "foreign", "autumn", "orandris", "our enjour", "northern juice", "orton julius",\
                       "orang joyce", "orang juiis", "orange joyce", "arun jus", "jordan jus"], # removed 'order' and all "just orange" so there is no conflict with orange 
    "7up":            ["seven", "up", "7", "nope", "devanab", "savannah", "sabona", "sabanab"], # removed o "simon" cause it conflicts with th name Simon
    "Water":          ["water", "laude"]
}

foods_dict = {
    "Tuna":             ["tuna"],
    "Tomato_Soup":      ["tomato"],
    "Spam":             ["spam", "spum", "spun"],
    "Mustard":          ["mustard", "musta", "mustn't", "moustache"],    
    "Strawberry_Jello": ["strawberry jello"],
    "Chocolate_Jello":  ["chocolat"],
    "Coffee_Grounds":   ["coffee", "grounds"],
    "Sugar":            ["sugar"],

    "Pear":             ["pear", "pair", "pier", "beer", "bare"],
    "Plum":             ["plum", "clump", "blam", "blum", "plumb"],
    "Peach":            ["peach"],
    "Lemon":            ["lemon"],
    "Orange":           ["orange", "orton"],
    "Strawberry":       ["strawberr"],
    "Banana":           ["banana"],
    "Apple":            ["apple"],

    "Pringles":         ["pringle", "sprinkles", "wrinkles", "bringles", "friendos", "pre-ingles", "pringos", "bring those", "prueingles"],
    "Cornflakes":       ["cornflakes", "corn flakes", "conflicts", "gonna flex"],
    "Cheezit":          ["cheese", "cheezit", "she's it", "shisit"]
}

numbers_dict = {
    "Zero":  ["0", "zero"],
    "One":   ["1", "one"],
    "Two":   ["2", "two"],
    "Three": ["3", "three"],
    "Four":  ["4", "four"],    
    "Five":  ["5", "five"],
    "Six":   ["6", "six"],
    "Seven": ["7", "seven"],
    "Eight": ["8", "eight"],
    "Nine":  ["9", "nine"]   
}

stop_dict = {
	"stop": ["stop", "arrive", "done"]
}

yes_no_dict = {
	"yes": ["yes", "affirmative", "yup", "ok"],
	"no":  ["no", "negative", "incorrect"]
}
# no already includes nope and not, since both these have 'no' in their words

charmie_dict = {
	"CHARMIE": ["robot", "sharmie", "sharmy", "sharami", "sharpie", "charlie", "tommy", "shahirmi", "sho",\
                "charmy", "sharmic", "shirely", "shardomie", "sharmik", "charmic", "xiaomi", "sharme",\
                "sharmi", "shower me", "shire me", "shatami", "chardon me", "shard me", "share me", "show me",\
                "sharm", "charm", "shower me", "charlotte", "shout me", "shanna me", "shawnee", "charmida",\
                "sure me", "ernie", "shar me", "sharp knee", "sharp me", "sharpening", "shireby", "jeremy",\
                "army", "sharp", "army", "mean", "shot me", "shahdami", "shai to me", "shot of me", "shimey"\
                "chardomie", "sir me", "shah mee", "sir", "shut up", "shahamid", "sha",  "shot", "shod", "shu"],
	"SPARK":   ["spark", "park",  "parth", "tiago", "diego"]
}



"""   ---   from RoboCup23 Bordeaux, France   ---

names_dict = {
    "Adel":     ["adel", "adele", "adam", "avel"],
    "Angel":    ["angel", "anjo", "hell", "ahil", "anjou", "anjal", "hong kong", "zangal" ],
    "Axel":     ["axel", "oxford", "aksal", "hotel", "axo", "oxel, oxyl", "axil", "unhail", "oxo"],
    "Charlie":  ["charlie", "shire lee", "sean", "shoutily", "chorley", "cherry"],
    "Jane":     ["jane", "jan", "jain", "jame", "jani", "jainie"],
    "John":     ["john", "jhong", "joe", "jopne"],
    "Jules":    ["jules", "jews", "drews", "jewels", "julie"],
    "Morgan":   ["morgan", "modern", "mardigan", "morgana", "mokkan", "maud", "modogan"],
    "Paris":    ["paris", "berries", "ferries", "parish", "pari", "hari", "barry", "patrice", "paix"],
    "Robin":    ["robin", "harbin", "robyn", "hobby", "hobean", "hohvin", "robon", "corbin", "haubin", "horven"],
    "Simone":   ["simone", "simon", "simo", "siman"]
}

drinks_dict = {
    "Cola":          [" cola", "coke", "coca", "coconut", "cook", "cok", "color", "call", "gog", "koch", "kullum", "goog"],
    "OrangeJuice":   ["orange", "orangous", "orangels", "foreign", "autumn", "orton", "orandris", "order"],
    "Milk":          ["milk", "muke", "mew", "milburn", "mio", "mieuki", "mute", "mille", "mieok"],
	"IceTea":        [" ice", "tea", "ist", "i-st", "stick", "i see", "i still", "i stick", "ice cream", "icedy",\
                    "icedi", "istie", "isti", "isalutti", "hasty", "i sati", "i see", "istia", "i steve", "i stay",\
                    "an i.c.", "anasti", "aussie", "aesti", "esti", "nice d", "ict"],
    "JuicePack":     ["pack", "gispeck", "druspak", "juspek", "swissbag", "speck", "just play", "packing", "becky",\
                      "bagging", "pachy", "peky", "paki", "peggy", "bek", "bake", "beck", "back", "speak", "bank"],
    "RedWine":       ["red", "wine", "edvin", "royce", "white", "ready", "headwind"],
    "TropicalJuice": ["tropical", "e-copic", "trophy dungeons", "tropi"]
}

foods_dict = {
    "Tuna":  ["tuna"],
    "TomatoSoup": ["tomato"],
    "Spam":  ["spam"],
    "Mustard": ["mustard"],    
    # "StrawberryJello": ["strawberry"],
    "ChocolateJello": ["chocolat", "jello"],
    "CofeeGrounds": ["coffee", "grounds"],

    "Pear": ["pear", "pair"],
    "Plum": ["plum"],
    "Peach": ["peach"],
    "Lemon": ["lemon"],
    # "Orange": ["orange"],
    "Banana": ["banana"],
    "Apple": ["apple"],
    "Strawberry": ["strawberr"],

    "Pringles": ["pringles"],
    "Cornflakes": ["cornflakes"],
    "Cheezit": ["cheese", "cheezit"]

}
"""


"""   ---   from FNR23 Tomar, Portugal   ---

names_dict = {
    "Amelia":    ["amelia", "emilia"],
    "Angel":     ["angel"],
    "Ava":       ["ava", "eva", "abba"],
    "Charlie":   ["charlie", "shire lee", "sean"],
    "Charlotte": ["charlotte", "char law", "sharhr law", "shah law"],
    "Hunter":    ["hunter", "anther", "aunsel", "antard", "ulthur", "ontoad", "untired", "ansel", "andrew", "aunt", "anto"],
    "Max":       ["max"],
    "Mia":       ["mia"],
    "Olivia":    ["olivia", "ali via", "olivea"],
    "Parker":    ["parker", "parkel", "parko", "bart", "park"],
    "Sam":       ["sam", "same", "some"],
    "Jack":      ["jack", "jake", "zack", "zick", "zac"],
    "Noah":      ["noah", "nore", "no", "noa"],
    "Oliver":    ["oliver", "olive her"],
    "Thomas":    ["thomas", "two miles"],
    "William":   ["william"]
}

drinks_dict = {
    "7up": ["seven", "up", "7", "simon", "nope", "devanab", "savannah", "sabona"],
	"IceTea": ["ice", "tea", "ist", "i-st", "stick", "i see", "i still", "i stick", "ice cream", "icedy",\
               "icedi", "istie", "isti", "isalutti", "hasty", "i sati", "i see", "istia", "i steve", "i stay",\
               "an i.c.", "anasti", "aussie", "aesti", "esti"]
}
"""