import pickle
import torch
from fxpmath import Fxp
import torch
from torchvision import transforms, datasets
import pickle


def tohex(val, nbits):
    return hex((val + (1 << nbits)) % (1 << nbits))

state_dict = None

bits = 16
frac_bits = 11

with open("MLP.state_dict","rb") as stateFile:
    state_dict = pickle.load(stateFile)
    last = index = 0
    for param_tensor in state_dict:
        print(param_tensor, "\t", state_dict[param_tensor].size())
        fileName = param_tensor.replace(".","_") + "_" + str(bits) + ".mem"
        with open(fileName, "w") as out:
            for param in state_dict[param_tensor]:
                if (param.size() != torch.Size([])):
                    for subParam in param:
                        val = tohex(Fxp(subParam.item(), n_int=bits-frac_bits ,n_frac=frac_bits, signed=True).raw(), bits)
                        out.write(str(val).replace("0x","")+" ")
                else:
                    val = tohex(Fxp(param.item(), n_int=bits-frac_bits ,n_frac=frac_bits, signed=True).raw(), bits)
                    out.write(str(val).replace("0x","")+"\n")

test = datasets.MNIST("", train=True, download=False, transform = transforms.Compose([transforms.ToTensor()]))
testset = torch.utils.data.DataLoader(test, batch_size=10, shuffle=False)
count = 1
amount = 1
index = 0
photoSet = []
with torch.no_grad():
    for data in testset:
        X, y = data
        pic = X.view(-1, 784)
        array = []
        for param in pic:
            for subParam in param:
                if not index<784:
                    photoSet.append(array)
                    array = []
                    index = 0
                val = tohex(Fxp(subParam.item(), n_int=bits-frac_bits ,n_frac=frac_bits, signed=True).raw(), bits)
                array.append(str(val).replace("0x",""))
                index += 1
        photoSet.append(array)
        count += 1
        if (count > amount): break

    index = 0
    with open("answers.txt", "w") as answers:
        for photo in photoSet:
            index += 1
            fileName = "picture_" + str(index) + "_" + str(bits) + ".mem"
            answers.write(str(y[index-1].item())+" ")
            print(str(y[index-1].item())+" " + fileName)
            with open(fileName, "w") as out:
                for pixel in photo:
                    out.write(pixel + " ")