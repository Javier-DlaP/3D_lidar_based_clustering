from bs4 import BeautifulSoup as bs
import numpy as np

class Object3D:
    def __init__(self, objectType, h, w, l, tx, ty, tz, rx, ry, rz):
        self.objectType = str(objectType)
        self.h = float(h)
        self.w = float(w)
        self.l = float(l)
        self.tx = float(tx)
        self.ty = float(ty)
        self.tz = float(tz)
        self.rx = float(rx)
        self.ry = float(ry)
        self.rz = float(rz)

    def __str__(self):
        return "type_:"+self.objectType+", h:"+str(self.h)+", w:"+str(self.w)+", l:"+str(self.l)+ \
                ",\ntx:"+str(self.tx)+", ty:"+str(self.ty)+", tz:"+str(self.tz)+ \
                ",\nrx:"+str(self.rx)+", ry:"+str(self.ry)+", rz:"+str(self.rz)

def get_objects(id_frame):

    objects3D = []

    with open("tracklet_labels.xml") as xml:
        data = xml.read()
        soup = bs(data, "xml")
        xml_tracklets = soup.find('tracklets')
        xml_items_1 = xml_tracklets.find_all('item', recursive=False)

        # iterate over every object tracked
        for xml_item_1 in xml_items_1:
            first_frame = int(xml_item_1.find('first_frame').text)

            # if the object has been tracked in the frame we are looking for
            if(first_frame <= id_frame):
                objectType, h, w, l = [xml_item_1.find('objectType').text, xml_item_1.find('h').text, xml_item_1.find('w').text, xml_item_1.find('l').text]
                xml_items_2 = xml_item_1.find_all('item')
                if(len(xml_items_2) > id_frame - first_frame):

                    # go to the frame
                    xml_item_2 = xml_items_2[id_frame - first_frame]
                    tx, ty, tz, rx, ry, rz = [xml_item_2.find('tx').text, xml_item_2.find('ty').text,xml_item_2.find('tz').text,
                                              xml_item_2.find('rx').text, xml_item_2.find('ry').text, xml_item_2.find('rz').text]
                    object3D = Object3D(objectType, h, w, l, tx, ty, tz, rx, ry, rz)
                    objects3D.append(object3D)

    return objects3D

if __name__ == "__main__":
    get_objects(1)