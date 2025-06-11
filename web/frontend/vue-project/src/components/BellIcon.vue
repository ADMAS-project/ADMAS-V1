<script setup>
import { FontAwesomeIcon } from '@fortawesome/vue-fontawesome'
import { ref } from 'vue'

const props = defineProps({
  count: Number,
  carData: [{
    carUUID: String,
    langitude: Number,
    latitude: Number,
    driverStatus: String,
  }]
})

const show = ref(false)

function showNotificationMenu() {
  show.value = !show.value
}
</script>

<template>
      <div>
        <font-awesome-icon :icon="['fas', 'bell']" size="2xl" class="
          text-slate-900
          cursor-pointer
          hover:!text-slate-700
          " @click="showNotificationMenu" />
          <p v-if="props.count != 0" class="bg-red-500 text-white text-center rounded-full px-1 absolute top-1 text-sm w-fit">{{ props.count }}</p>
          <div v-if="show==true" class="
            bg-gray-100
            p-1
            m-0
            w-20
            absolute
            top-11
            right-5
            border
            border-black
            rounded
            max-h-32
            overflow-scroll
            "
            id="notification-container"
            >
            <ul>
              <li id="notification-item" v-for="car in props.carData"><div class="
                  hover:bg-gray-300
                  cursor-pointer
                  p-1
                  ">
                  <div>{{ car.carUUID }}</div>
                  <div class="flex justify-between">
                    <a :href="'/'+car.carUUID">
                    <button class="
                      bg-green-400
                      px-1
                      rounded
                      hover:bg-green-600
                      hover:text-white
                      mt-1
                      mr-2
                      ">Show</button>
                    </a>
                    <button class="
                      bg-red-400
                      px-1
                      rounded
                      hover:bg-red-600
                      hover:text-white
                      mt-1
                      ">Dismiss</button>
                  </div>
                </div>
              </li>
            </ul>
          </div>
      </div>
</template>

<style scoped>
#notification-container {
  width: 200px;
}
#notification-item {
  font-size: 12px;
}
</style>
